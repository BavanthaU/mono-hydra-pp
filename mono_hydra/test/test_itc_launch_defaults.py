"""Check ITC component defaults and routing without starting ROS processes.

Run in a sourced workspace with:
  python3 -m unittest discover -s src/mono_hydra/test -v
"""
import importlib.util
from pathlib import Path
import unittest

import yaml
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.utilities import normalize_to_list_of_substitutions, perform_substitutions
from launch_ros.actions import Node
from launch_ros.utilities import evaluate_parameters

LAUNCH_DIR = Path(__file__).resolve().parents[1] / "launch"


def load(name):
    spec = importlib.util.spec_from_file_location(name, LAUNCH_DIR / name)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def resolve(context, value):
    return perform_substitutions(context, normalize_to_list_of_substitutions(value))


class ItcLaunchTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.itc = load("mono_hydra_itc_rosbag.launch.py")
        cls.shared = load("mono_hydra_scannet.launch.py")

    def prepare(self, overrides=None):
        context = LaunchContext()
        context.launch_configurations.update(overrides or {})
        top = self.itc.generate_launch_description()
        for action in top.entities:
            if isinstance(action, DeclareLaunchArgument):
                action.execute(context)
        self.itc._validate_weight_preset(context)
        include = next(a for a in top.entities if isinstance(a, IncludeLaunchDescription))
        forwarded = {key: resolve(context, value) for key, value in include.launch_arguments}
        context.launch_configurations.update(forwarded)
        shared = self.shared.generate_launch_description()
        for action in shared.entities:
            if isinstance(action, DeclareLaunchArgument):
                action.execute(context)
        return context, shared

    def params(self, context, node):
        result = {}
        for item in evaluate_parameters(context, node._Node__parameters):
            if isinstance(item, dict):
                result.update(item)
        return result

    def node(self, context, description, executable):
        return next(a for a in description.entities if isinstance(a, Node)
                    and resolve(context, a.node_executable) == executable)

    def test_paper_defaults_reach_nodes(self):
        context, description = self.prepare()
        config = context.launch_configurations
        self.assertEqual(config['rvio2_variant'], 'superpoint')
        self.assertEqual(config['perception_backend'], 'torch')
        self.assertEqual(config['use_ptc_refinement'], 'false')
        for key in ('use_temporal_alignment', 'temporal_fuse_labels',
                    'use_sparse_depth_factors', 'revision_fusion_weighting',
                    'revision_object_association', 'use_semantic_masking', 'enable_kimera_lcd'):
            self.assertEqual(config[key], 'true', key)
        temporal = self.shared._temporal_alignment_node(context)
        self.assertEqual(len(temporal), 1)
        self.assertTrue(self.params(context, temporal[0])['fuse_labels'])
        kimera = self.params(context, self.node(context, description, 'mono_hydra_vio_ros_node'))
        feature = self.params(context, self.node(context, description, 'vio_feature_interface_node'))
        self.assertTrue(kimera['use_sparse_depth_factors'])
        self.assertTrue(kimera['use_lcd'])
        self.assertEqual(config['enable_lcd'], 'false')
        self.assertTrue(feature['enable_sparse_depth'])
        self.assertEqual(kimera['depth_cam_topic'], config['temporal_depth_topic'])
        self.assertEqual(feature['dense_depth_topic'], config['temporal_depth_topic'])
        self.assertEqual(self.shared._ptc_refinement_node(context), [])
        hydra = self.shared._official_hydra_group(context)[0]
        include = next(a for a in hydra.get_sub_entities() if isinstance(a, IncludeLaunchDescription))
        arguments = {key: resolve(context, value) for key, value in include.launch_arguments}
        extra = yaml.safe_load(arguments['extra_yaml'])
        self.assertTrue(extra['active_window']['geometric_reliability']['enabled'])
        self.assertTrue(extra['frontend']['objects']['use_temporal_overlap'])
        self.assertTrue(extra['backend']['update_functors']['objects']['require_mesh_overlap'])

    def test_disable_components_routes_raw_depth(self):
        context, description = self.prepare({key: 'false' for key in (
            'use_temporal_alignment', 'use_sparse_depth_factors', 'revision_fusion_weighting',
            'revision_object_association', 'temporal_fuse_labels')})
        self.assertEqual(self.shared._temporal_alignment_node(context), [])
        kimera = self.params(context, self.node(context, description, 'mono_hydra_vio_ros_node'))
        self.assertFalse(kimera['use_sparse_depth_factors'])
        self.assertEqual(kimera['depth_cam_topic'], context.launch_configurations['depth_topic'])
        feature = self.params(context, self.node(context, description, 'vio_feature_interface_node'))
        self.assertFalse(feature['enable_sparse_depth'])

    def test_ptc_replaces_temporal_alignment(self):
        for references in ('1', '2'):
            with self.subTest(references=references):
                context, _ = self.prepare({'use_ptc_refinement': 'true',
                                          'use_temporal_alignment': 'false',
                                          'ptc_references': references})
                self.assertEqual(self.shared._temporal_alignment_node(context), [])
                ptc = self.shared._ptc_refinement_node(context)
                self.assertEqual(len(ptc), 1)
                self.assertEqual(self.params(context, ptc[0])['references'], int(references))

    def test_incompatible_refiners_are_rejected(self):
        context, _ = self.prepare({'use_ptc_refinement': 'true'})
        with self.assertRaisesRegex(RuntimeError, 'disable use_temporal_alignment'):
            self.shared._ptc_refinement_node(context)

    def test_scene_presets_and_classical_fallback(self):
        for dataset in ('itc', 'scannet', 'nyudv2'):
            with self.subTest(dataset=dataset):
                context, description = self.prepare({'perception_dataset': dataset,
                                                     'rvio2_variant': 'classical'})
                self.assertEqual(context.launch_configurations['perception_dataset'], dataset)
                classical = self.node(context, description, 'rvio2_mono_node')
                superpoint = self.node(context, description, 'rvio2_sp_mono_node')
                self.assertTrue(classical.condition.evaluate(context))
                self.assertFalse(superpoint.condition.evaluate(context))


if __name__ == '__main__':
    unittest.main()
