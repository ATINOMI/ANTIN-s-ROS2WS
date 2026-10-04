#!/usr/bin/env python3
"""Read-only runtime ownership and original-map audit for the isolated demo."""
import argparse
import hashlib
import json
from pathlib import Path
import time

import rclpy
from rclpy.node import Node
from tf2_msgs.msg import TFMessage
from std_msgs.msg import Bool
from geometry_msgs.msg import PoseWithCovarianceStamped
import yaml


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--tf-ownership-file', type=Path, required=True,
                        help='JSON captured by the rclcpp message-GID audit')
    args = parser.parse_args()
    rclpy.init()
    node = Node('scurm_tf_ownership_audit')
    seen, valid, pose = set(), [], {}

    def transforms(msg):
        for tf in msg.transforms:
            seen.add(tf.header.frame_id + '->' + tf.child_frame_id)

    def localization(msg):
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        import math
        pose['xy_yaw'] = [p.x, p.y, math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))]
        pose['stamp'] = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

    subscriptions = [node.create_subscription(TFMessage, '/tf', transforms, 100),
                     node.create_subscription(Bool, '/mini_nav/localization_valid', lambda m: valid.append(m.data), 10),
                     node.create_subscription(PoseWithCovarianceStamped, '/scurm/localization_pose', localization, 10)]
    end = time.monotonic() + 3
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=.05)
    result = {'ros_domain_id': 227, 'gz_partition': 'scurm_mini_nav',
              'nodes': sorted(name for name, _ in node.get_node_names_and_namespaces()),
              'tf_frames': sorted(seen),
              'tf_publisher_gids': json.loads(args.tf_ownership_file.read_text()),
              'tf_ownership_method': 'Count distinct rclcpp MessageInfo publisher_gid values per parent/child frame',
              'cmd_vel_publishers': [p.node_name for p in node.get_publishers_info_by_topic('/cmd_vel')],
              'quality_samples': len(valid), 'quality_all_valid': bool(valid) and all(valid),
              'settled_pose': pose, 'gazebo_servers': []}
    for proc in Path('/proc').glob('[0-9]*'):
        try:
            cmd = (proc / 'cmdline').read_bytes().split(b'\0')
            words = (proc / 'cmdline').read_bytes().replace(b'\0', b' ').split()
            if words[:2] == [b'gz', b'sim'] and b'-s' in words:
                env = (proc / 'environ').read_bytes().split(b'\0')
                if b'GZ_PARTITION=scurm_mini_nav' in env:
                    result['gazebo_servers'].append(int(proc.name))
        except OSError:
            pass
    root = Path(__file__).resolve().parents[1]
    manifest = json.loads((root / 'scurm_sim/maps/localization_manifest.json').read_text())
    map_path = Path(manifest['source_map'])
    image = map_path.parent / yaml.safe_load(map_path.read_text())['image']
    result['original_2d_image_unchanged'] = hashlib.sha256(image.read_bytes()).hexdigest() == manifest['source_image_sha256']
    result['passed'] = (result['cmd_vel_publishers'] == ['velocity_guard'] and len(result['gazebo_servers']) == 1
                        and result['quality_all_valid'] and result['original_2d_image_unchanged']
                        and len(result['tf_publisher_gids'].get('map->odom', [])) == 1
                        and len(result['tf_publisher_gids'].get('odom->base_footprint', [])) == 1)
    args.output.write_text(json.dumps(result, indent=2) + '\n')
    print(json.dumps(result, indent=2))
    node.destroy_node()
    rclpy.shutdown()
    raise SystemExit(0 if result['passed'] else 1)


if __name__ == '__main__':
    main()
