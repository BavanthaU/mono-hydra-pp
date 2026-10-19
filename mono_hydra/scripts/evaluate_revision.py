#!/usr/bin/env python3
"""Non-overwriting mesh, instance-support and runtime evaluation.

Requires numpy/scipy and the existing evaluation environment's Open3D. Distances
are between deterministic area-uniform surface samples (an approximation to
continuous surface distance). No scale fitting, clipping of errors, or ICP.
Supply a fixed rigid transform derived independently of the scored surface.
"""
import argparse
from collections import Counter, defaultdict
import csv
import hashlib
import json
from pathlib import Path

import numpy as np
from scipy.spatial import cKDTree


def distribution(values):
    a = np.asarray(values, dtype=float)
    a = a[np.isfinite(a)]
    if not len(a):
        return {'samples': 0}
    return dict(samples=len(a), mean=float(a.mean()), median=float(np.median(a)),
                p95=float(np.percentile(a, 95)), p99=float(np.percentile(a, 99)), maximum=float(a.max()))


def sample_mesh(mesh, count, rng):
    v, f = np.asarray(mesh.vertices), np.asarray(mesh.triangles)
    if not len(v) or not len(f) or not np.isfinite(v).all():
        raise ValueError('Expected nonempty finite triangle mesh')
    if f.min() < 0 or f.max() >= len(v):
        raise ValueError('Invalid mesh face indices')
    a = v[f[:, 0]]; b = v[f[:, 1]]; c = v[f[:, 2]]
    areas = np.linalg.norm(np.cross(b-a, c-a), axis=1)
    if not np.isfinite(areas).all() or areas.sum() <= 0:
        raise ValueError('Invalid/degenerate mesh area')
    chosen = rng.choice(len(f), count, p=areas/areas.sum())
    u = np.sqrt(rng.random(count)); w = rng.random(count)
    return (1-u[:, None])*a[chosen] + (u*(1-w))[:, None]*b[chosen] + (u*w)[:, None]*c[chosen]


def instance_support(pred_vertices, objects, gt_vertices, gt_instances, tolerance=0.1,
                     min_fraction=0.2, min_points=10):
    tree = cKDTree(gt_vertices)
    gt_to_pred = defaultdict(list); merged = []; details = []
    for node in objects:
        indices = np.unique(node['attributes']['mesh_connections']).astype(int)
        if len(indices) and (indices.min() < 0 or indices.max() >= len(pred_vertices)):
            raise ValueError('DSG mesh indices do not match supplied predicted mesh')
        if not len(indices):
            continue
        distance, closest = tree.query(pred_vertices[indices], workers=2)
        observed = gt_instances[closest[distance <= tolerance]]
        counts = Counter(int(x) for x in observed if x >= 0)
        supported = [instance for instance, n in counts.items()
                     if n >= min_points and n/len(indices) >= min_fraction]
        for instance in supported:
            gt_to_pred[instance].append(node['id'])
        if len(supported) > 1:
            merged.append(node['id'])
        details.append(dict(predicted_id=node['id'], gt_support=supported, supported_vertices=dict(counts)))
    return dict(scope='Geometric instance support, not temporal ID switches; thresholds explicit.',
        tolerance_m=tolerance, min_fraction=min_fraction, min_points=min_points,
        annotated_gt_instances=len(set(int(x) for x in gt_instances if x >= 0)),
        matched_gt_instances=len(gt_to_pred),
        fragmented_gt_instances=sum(len(ids) > 1 for ids in gt_to_pred.values()),
        excess_predicted_instances=sum(max(0, len(ids)-1) for ids in gt_to_pred.values()),
        false_merge_candidates=len(merged), false_merge_ids=merged,
        gt_to_pred=dict(gt_to_pred), per_prediction=details)


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--run', type=Path, required=True, help='Hydra export containing backend/ and timing/')
    ap.add_argument('--output', type=Path, required=True)
    ap.add_argument('--gt-mesh', type=Path)
    ap.add_argument('--gt-segments', type=Path)
    ap.add_argument('--gt-aggregation', type=Path)
    ap.add_argument('--transform', type=Path, help='4x4 rigid pred-to-GT transform; identity otherwise')
    ap.add_argument('--samples', type=int, default=100000)
    ap.add_argument('--seed', type=int, default=31)
    ap.add_argument('--thresholds', type=float, nargs='+', default=[0.05, 0.1])
    ap.add_argument('--instance-tolerance', type=float, default=0.1)
    args = ap.parse_args()
    if args.output.exists():
        ap.error('Output exists; choose a new filename')
    if args.samples < 100 or min(args.thresholds) <= 0 or args.instance_tolerance <= 0:
        ap.error('Invalid sample count or distance thresholds')
    import open3d as o3d
    pred_path = args.run/'backend/mesh.ply'
    pred = o3d.io.read_triangle_mesh(str(pred_path))
    vertices = np.asarray(pred.vertices)
    transform = np.eye(4) if args.transform is None else np.loadtxt(args.transform)
    if (transform.shape != (4, 4) or not np.isfinite(transform).all() or
            not np.allclose(transform[3], [0, 0, 0, 1]) or
            not np.allclose(transform[:3, :3].T@transform[:3, :3], np.eye(3), atol=1e-4) or
            not np.isclose(np.linalg.det(transform[:3, :3]), 1, atol=1e-4)):
        raise ValueError('Evaluation transform must be rigid SE(3); scale fitting is prohibited here')
    result = dict(run=str(args.run.resolve()), transform=transform.tolist(), seed=args.seed,
                  alignment='Provided rigid transform' if args.transform else 'Identity (caller must verify common frame)')
    result['mesh'] = dict(vertices=len(vertices), triangles=len(pred.triangles), finite=bool(np.isfinite(vertices).all()))
    graph = args.run/'backend/dsg.json'
    objects = []
    if graph.exists():
        objects = [n for n in json.loads(graph.read_text())['nodes'] if n['attributes'].get('type') == 'ObjectNodeAttributes']
        owners = Counter(v for n in objects for v in set(n['attributes']['mesh_connections']))
        result['objects'] = dict(count=len(objects), by_label=dict(Counter(n['attributes']['semantic_label'] for n in objects)),
                                 vertices_with_multiple_owners=sum(n > 1 for n in owners.values()))
    if args.gt_mesh:
        gt = o3d.io.read_triangle_mesh(str(args.gt_mesh))
        gt_points = sample_mesh(gt, args.samples, np.random.default_rng(args.seed))
        pred_points = sample_mesh(pred, args.samples, np.random.default_rng(args.seed))
        pred_points = pred_points@transform[:3, :3].T + transform[:3, 3]
        accuracy = cKDTree(gt_points).query(pred_points, workers=2)[0]
        completeness = cKDTree(pred_points).query(gt_points, workers=2)[0]
        result['geometry'] = dict(scope='Whole supplied GT mesh; provide observed-region GT for partial trajectories.',
            metric='Area-uniform sampled bidirectional surface distances in meters', samples_per_surface=args.samples,
            accuracy_m=distribution(accuracy), completeness_m=distribution(completeness), thresholds={})
        for threshold in args.thresholds:
            precision, recall = float(np.mean(accuracy <= threshold)), float(np.mean(completeness <= threshold))
            result['geometry']['thresholds'][str(threshold)] = dict(precision=precision, recall=recall,
                f_score=2*precision*recall/(precision+recall) if precision+recall else 0.0)
        if bool(args.gt_segments) != bool(args.gt_aggregation):
            ap.error('Both instance segment and aggregation files are required')
        if args.gt_segments:
            segments = np.asarray(json.loads(args.gt_segments.read_text())['segIndices'])
            if len(segments) != len(gt.vertices):
                raise ValueError('Segmentation and GT mesh vertex counts differ')
            instances = np.full(len(segments), -1, dtype=np.int64)
            mapping = {}
            for group in json.loads(args.gt_aggregation.read_text())['segGroups']:
                for segment in group['segments']:
                    if segment in mapping and mapping[segment] != group['objectId']:
                        raise ValueError('Conflicting GT segment ownership')
                    mapping[segment] = group['objectId']
            instances[:] = [mapping.get(int(s), -1) for s in segments]
            result['instance_support'] = instance_support(vertices@transform[:3, :3].T+transform[:3, 3],
                objects, np.asarray(gt.vertices), instances, args.instance_tolerance)
    result['timing_ms'] = {}
    for file in sorted((args.run/'timing').glob('*_timing_raw.csv')):
        with file.open() as stream:
            rows = list(csv.DictReader(stream))
        result['timing_ms'][file.stem] = distribution([1000*float(r['elapsed(s)']) for r in rows])
    result['timing_scope'] = 'Per-stage events; nested timers must not be summed as pipeline throughput.'
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open('x') as out:
        json.dump(result, out, indent=2, allow_nan=False)
    print(args.output)


if __name__ == '__main__':
    main()
