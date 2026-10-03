#!/usr/bin/env python3
"""从固定上游源码生成无 ROS1 的 HBA CLI；保留 BA/PGO 数值流程。"""
import argparse
from pathlib import Path
import shutil

parser = argparse.ArgumentParser()
parser.add_argument('upstream', type=Path)
parser.add_argument('output', type=Path)
args = parser.parse_args()
args.output.mkdir(parents=True, exist_ok=True)
shutil.copytree(args.upstream / 'include', args.output / 'include', dirs_exist_ok=True)
shutil.copy(args.upstream / 'LICENSE', args.output / 'LICENSE')
clock = '#include <chrono>\ninline double wall_seconds() { return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count(); }\n'
(args.output / 'include/offline_clock.hpp').write_text(clock)
for name in ['ba.hpp', 'hba.hpp']:
    path = args.output / 'include' / name
    s = path.read_text()
    s = s.replace('#include <visualization_msgs/Marker.h>', '#include "offline_clock.hpp"' if name == 'ba.hpp' else '')
    s = s.replace('#include <visualization_msgs/MarkerArray.h>', '')
    s = s.replace('ros::Time::now().toSec()', 'wall_seconds()')
    if name == 'ba.hpp':
        s = s.replace('int thd_num = 16;', 'int thd_num = 4;')
        # 无有效平面时停止，不把无约束输出作为优化成功。
        s = s.replace('std::vector<double> residuals = voxhess.evaluate_residual(x_stats);',
                      'if (voxhess.plvec_voxels.size() < 10) throw std::runtime_error("Insufficient multi-frame plane constraints");\n    std::vector<double> residuals = voxhess.evaluate_residual(x_stats);')
    path.write_text(s)
path = args.output / 'include/mypcl.hpp'
s = path.read_text().replace('while(!file.eof())\n    {\n      file >> tx >> ty >> tz >> w >> x >> y >> z;',
                            'while(file >> tx >> ty >> tz >> w >> x >> y >> z)\n    {')
old = '\n'.join(f'      pose_vec[i].q.{c}() = (q0.inverse()*pose_vec[i].q).{c}();' for c in 'wxyz')
s = s.replace(old, '      pose_vec[i].q = (q0.inverse()*pose_vec[i].q).normalized();')
s = s.replace('file.open(path + "pose.json", std::ofstream::app);',
              'file.open(path + "pose.json", std::ofstream::app);\n    file << std::setprecision(17);')
path.write_text(s)
s = (args.upstream / 'source/hba.cpp').read_text()
for include in ['ros/ros.h', 'sensor_msgs/Imu.h', 'sensor_msgs/PointCloud2.h',
                'geometry_msgs/PoseArray.h', 'tf/transform_broadcaster.h', 'pcl_conversions/pcl_conversions.h']:
    s = s.replace('#include <' + include + '>', '')
s = s.replace('ros::Time::now().toSec()', 'wall_seconds()')
s = s[:s.index('int main(int argc, char** argv)')]
s += '''int main(int argc, char** argv) {
    if (argc != 4) { std::cerr << "Usage: hba_offline DATA_DIR VOXEL_SIZE DOWNSAMPLE\\n"; return 2; }
    try {
        std::string path = argv[1];
        if (path.back() != '/') path += '/';
        const auto poses = mypcl::read_pose(path + "pose.json");
        if (poses.size() < 35 || poses.size() > 500) throw std::runtime_error("HBA experiment requires 35..500 frames");
        for (size_t i = 0; i < poses.size(); ++i) {
            if (!poses[i].t.allFinite() || std::abs(poses[i].q.norm()-1.0)>1e-4) throw std::runtime_error("Invalid input pose");
        }
        HBA hba(2, path, 2);
        for (auto& layer : hba.layers) {
            layer.voxel_size = std::stod(argv[2]);
            layer.downsample_size = std::stod(argv[3]);
            layer.max_iter = 5;
        }
        distribute_thread(hba.layers[0], hba.layers[1]);
        hba.update_next_layer_state(0);
        global_ba(hba.layers[1]);
        hba.pose_graph_optimization();
        std::cout << "iteration complete\\n";
        return 0;
    } catch (const std::exception& error) { std::cerr << error.what() << '\\n'; return 1; }
}
'''
(args.output / 'hba_offline.cpp').write_text(s)
