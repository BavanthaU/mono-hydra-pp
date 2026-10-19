#!/usr/bin/env python3
"""Replay a captured synchronized prediction/pose bundle into isolated Hydra.

Run in the sourced ROS2 environment. This does not rerun perception or VIO;
use identical captured inputs to isolate mapping/refinement contributions.
"""
import argparse
import csv
import json
import os
from pathlib import Path
import signal
import subprocess
import time

import psutil


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--workspace', type=Path, required=True)
    ap.add_argument('--capture', type=Path, required=True)
    ap.add_argument('--output', type=Path, required=True)
    ap.add_argument('--variant', choices=['baseline', 'fusion', 'objects', 'combined'], required=True)
    ap.add_argument('--ptc', choices=['none', 'single', 'multi'], default='none')
    ap.add_argument('--ptc-python-path', default=os.environ.get('PTC_DEPTH_PYTHONPATH', ''))
    ap.add_argument('--domain', type=int, default=78)
    ap.add_argument('--rate', type=float, default=0.5)
    ap.add_argument('--drain-seconds', type=float, default=20)
    args = ap.parse_args()
    if not args.capture.exists() or args.rate <= 0 or args.drain_seconds < 0:
        ap.error('Existing capture, positive rate and nonnegative drain time required')
    args.output.mkdir(parents=True, exist_ok=False)
    ws, out = args.workspace.resolve(), args.output.resolve()
    env = os.environ.copy(); env['ROS_DOMAIN_ID'] = str(args.domain)
    env['OMP_NUM_THREADS'] = '2'
    if args.ptc_python_path:
        env['PYTHONPATH'] = args.ptc_python_path + os.pathsep + env.get('PYTHONPATH', '')
    override = {'active_window': {'geometric_reliability': {'enabled': args.variant in ('fusion', 'combined')}}}
    if args.variant in ('objects', 'combined'):
        override['frontend'] = {'objects': {'use_temporal_overlap': True}}
        override['backend'] = {'update_functors': {'objects': {'require_mesh_overlap': True}}}
    cmd = [str(ws/'install/hydra_ros/lib/hydra_ros/hydra_ros_node'),
           '--config-utilities-file', str(ws/'src/mono_hydra/config/hydra_itc_topics.yaml'),
           '--config-utilities-file', str(ws/'src/mono_hydra_perception/config/label_spaces/nyud20_config_label_space.yaml'),
           '--config-utilities-file', str(ws/'src/mono_hydra_utils/hydra_ros/hydra_ros/config/sinks/active_window_sinks.yaml')+'@active_window',
           '--config-utilities-yaml', json.dumps(dict(enable_lcd=False, exit_after_clock=False,
               odom_frame='odom', robot_frame='base_link_kimera', map_frame='map', glog_verbosity=2, log_path=str(out/'map'))),
           '--config-utilities-yaml', json.dumps(override), '--ros-args', '-r', '__node:=hydra', '-p', 'use_sim_time:=true']
    inputs = ['/mono_hydra_perception/synced/image_raw', '/camera/depth_cam/image_raw', '/camera/seg_cam/labels_argmax']
    if args.ptc != 'none':
        inputs = ['/mono_hydra_refinement/rgb', '/mono_hydra_refinement/depth', '/mono_hydra_refinement/labels']
    for topic, target in zip(['rgb/image_raw', 'depth_registered/image_rect', 'semantic/image_raw'], inputs):
        cmd += ['-r', 'hydra/input/camera/'+topic+':='+target]
    cmd += ['-r', 'hydra/input/camera/rgb/camera_info:=/mono_hydra_perception/synced/camera_info',
            '-r', '/mono_hydra_vio_ros/pose_graph:=/mono_hydra_vio_ros/pose_graph_incremental']
    refcmd = [str(ws/'install/mono_hydra_perception/lib/mono_hydra_perception/ptc_refinement_node'),
              '--ros-args', '-p', 'use_sim_time:=true', '-p', 'references:='+('2' if args.ptc == 'multi' else '1'),
              '-p', 'metrics_path:='+str(out/'ptc_metrics.jsonl')]
    play = ['ros2', 'bag', 'play', str(args.capture.resolve()), '--rate', str(args.rate), '--delay', '3', '--topics',
            '/tf_static', '/tf', '/clock', '/mono_hydra_vio/odometry', '/camera/depth_cam/image_raw',
            '/camera/seg_cam/labels_argmax', '/mono_hydra_perception/synced/image_raw',
            '/mono_hydra_perception/synced/camera_info', '/mono_hydra_vio_ros/pose_graph_incremental']
    (out/'commands.json').write_text(json.dumps(dict(hydra=cmd, refiner=refcmd if args.ptc != 'none' else None,
        playback=play, variant=args.variant, ptc=args.ptc, domain=args.domain, rate=args.rate), indent=2))
    processes = []; files = []; cpu = {}; start = time.monotonic(); completed = False
    def launch(command, name):
        log = (out/(name+'.log')).open('x'); files.append(log)
        proc = subprocess.Popen(command, env=env, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        processes.append(proc); cpu[proc.pid] = psutil.Process(proc.pid)
        cpu[proc.pid].cpu_percent(None)
        return proc
    usage = (out/'resources.csv').open('x'); files.append(usage)
    writer = csv.writer(usage); writer.writerow(['wall_s', 'pid', 'cpu_percent_one_core_100', 'rss_bytes'])
    hydra = refiner = player = None
    error = None
    try:
        hydra = launch(cmd, 'hydra')
        refiner = launch(refcmd, 'ptc') if args.ptc != 'none' else None
        time.sleep(2)
        player = launch(play, 'play')
        ended = None
        while True:
            if hydra.poll() is not None or (refiner and refiner.poll() is not None):
                raise RuntimeError('Mapping/refinement process exited before drain completed')
            if player.poll() is not None:
                if player.returncode != 0:
                    raise RuntimeError('Playback failed')
                ended = ended or time.monotonic()
                if time.monotonic()-ended >= args.drain_seconds:
                    completed = True
                    break
            for proc in processes:
                try:
                    process = cpu[proc.pid]
                    writer.writerow([time.monotonic()-start, proc.pid, process.cpu_percent(None), process.memory_info().rss])
                except psutil.NoSuchProcess:
                    pass
            usage.flush()
            time.sleep(0.5)
    except BaseException as exc:
        error = repr(exc)
    finally:
        # Signal only process groups created by this runner. Preserve user sessions.
        for proc in reversed(processes):
            if proc.poll() is None:
                os.killpg(proc.pid, signal.SIGINT)
        for proc in processes:
            try:
                proc.wait(timeout=120)
            except subprocess.TimeoutExpired:
                error = (error or '') + f' Shutdown timeout pid={proc.pid}; left running for inspection.'
        (out/'run.json').write_text(json.dumps(dict(completed=completed and error is None,
            error=error, elapsed_s=time.monotonic()-start, returncodes=[p.poll() for p in processes],
            scope='Fixed prediction/pose mapping replay; not end-to-end throughput.'), indent=2))
        for f in files:
            f.close()
    if error or any(p.returncode != 0 for p in processes):
        raise SystemExit(error or 'Nonzero subprocess exit')


if __name__ == '__main__':
    main()
