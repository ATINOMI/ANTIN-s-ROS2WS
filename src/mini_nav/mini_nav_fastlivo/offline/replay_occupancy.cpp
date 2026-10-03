// 离线重放完整扫描的真实射线；只将车高端点作为碰撞占用证据。
#include <array>
#include <cmath>
#include <fstream>
#include <iostream>
#include <map>
#include <set>
#include <stdexcept>
#include <string>
#include <vector>
#include <Eigen/Geometry>
#include <octomap/OcTree.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

int main(int argc, char** argv) {
    if (argc != 3) {
        std::cerr << "Usage: replay_occupancy WORK_DIR VOXEL_RESOLUTION\n";
        return 2;
    }
    try {
        const std::string directory = argv[1];
        const double voxel = std::stod(argv[2]);
        if (!std::isfinite(voxel) || voxel < 0.01 || voxel > 0.1) {
            throw std::runtime_error("Invalid voxel resolution");
        }
        std::ifstream config(directory + "/projection.txt");
        double resolution, size, minimum, maximum;
        Eigen::Vector3d lidar, base;
        Eigen::Vector4d ground;
        if (!(config >> resolution >> size >> minimum >> maximum >> lidar.x() >> lidar.y() >> lidar.z()
                     >> base.x() >> base.y() >> base.z() >> ground[0] >> ground[1] >> ground[2] >> ground[3])) {
            throw std::runtime_error("Invalid projection config");
        }
        const int width = static_cast<int>(std::ceil(size / resolution));
        if (resolution <= 0 || width < 1 || width > 2000 || minimum >= maximum) {
            throw std::runtime_error("Invalid map bounds");
        }
        octomap::OcTree tree(voxel);
        tree.setProbHit(0.7);
        tree.setProbMiss(0.4);
        tree.setOccupancyThres(0.65);
        std::map<std::array<unsigned short, 3>, unsigned int> support;
        std::ifstream poses(directory + "/poses_map.txt");
        double tx, ty, tz, qw, qx, qy, qz;
        std::size_t frame = 0;
        while (poses >> tx >> ty >> tz >> qw >> qx >> qy >> qz) {
            const Eigen::Quaterniond q(qw, qx, qy, qz);
            if (!std::isfinite(q.norm()) || std::abs(q.norm()-1.0) > 1e-4) {
                throw std::runtime_error("Invalid replay quaternion");
            }
            const Eigen::Vector3d translation(tx, ty, tz);
            if (!translation.allFinite()) throw std::runtime_error("Invalid replay translation");
            const Eigen::Vector3d sensor = q * lidar + translation;
            const Eigen::Vector3d robot = q * base + translation;
            const octomap::point3d origin(sensor.x(), sensor.y(), sensor.z());
            pcl::PointCloud<pcl::PointXYZ> cloud;
            if (pcl::io::loadPCDFile(directory + "/pcd/" + std::to_string(frame) + ".pcd", cloud) < 0) {
                throw std::runtime_error("Missing full local scan");
            }
            octomap::KeySet free, occupied;
            octomap::KeyRay ray;
            for (const auto& value : cloud) {
                const Eigen::Vector3d point = q * Eigen::Vector3d(value.x, value.y, value.z) + translation;
                if (!point.allFinite() || (point-sensor).norm() > 15.0 ||
                    (point-robot).head<2>().norm() < 0.28) {
                    continue;
                }
                const octomap::point3d endpoint(point.x(), point.y(), point.z());
                if (!tree.computeRayKeys(origin, endpoint, ray)) {
                    throw std::runtime_error("Ray outside octree capacity");
                }
                free.insert(ray.begin(), ray.end());
                const double height = ground.head<3>().dot(point) + ground[3];
                octomap::OcTreeKey key;
                if (!tree.coordToKeyChecked(endpoint, key)) {
                    throw std::runtime_error("Endpoint outside octree capacity");
                }
                if (height > minimum && height <= maximum) {
                    occupied.insert(key);
                } else if (std::abs(height) <= 0.008) {
                    // 已确认地面只提供自由证据，避免地板体素跨越2cm边界。
                    free.insert(key);
                }
            }
            for (const auto& key : occupied) {
                free.erase(key);
                tree.updateNode(key, true, true);
                ++support[{key[0], key[1], key[2]}];
            }
            for (const auto& key : free) {
                tree.updateNode(key, false, true);
            }
            ++frame;
        }
        if (!poses.eof() || frame < 15) {
            throw std::runtime_error("Invalid or insufficient replay poses");
        }
        tree.updateInnerOccupancy();
        // 仅在既有平地场景中采用 observed-free 投影，不能证明整根车高柱可通行。
        std::vector<unsigned char> occupied(width*width, 0), candidate(width*width, 0), free(width*width, 0);
        const double origin_xy = -width * resolution / 2.0;
        for (auto leaf = tree.begin_leafs(); leaf != tree.end_leafs(); ++leaf) {
            const Eigen::Vector3d point(leaf.getX(), leaf.getY(), leaf.getZ());
            const double height = ground.head<3>().dot(point) + ground[3];
            if (height + voxel/2 < minimum || height - voxel/2 > maximum) {
                continue;
            }
            // 以体素面积投影到二维格，不能只用中心漏掉格边的低障碍。
            const double half = leaf.getSize() / 2.0;
            const int x0 = static_cast<int>(std::floor((point.x()-half-origin_xy+1e-9) / resolution));
            const int y0 = static_cast<int>(std::floor((point.y()-half-origin_xy+1e-9) / resolution));
            const int x1 = static_cast<int>(std::ceil((point.x()+half-origin_xy-1e-9) / resolution))-1;
            const int y1 = static_cast<int>(std::ceil((point.y()+half-origin_xy-1e-9) / resolution))-1;
            if (x0 < 0 || x1 >= width || y0 < 0 || y1 >= width) {
                throw std::runtime_error("Observed map exceeds output bounds");
            }
            const auto key = leaf.getKey();
            const double probability = leaf->getOccupancy();
            const auto observation = support.find({key[0], key[1], key[2]});
            const unsigned int hit_count = observation == support.end() ? 0 : observation->second;
            for (int y = y0; y <= y1; ++y) {
                for (int x = x0; x <= x1; ++x) {
                    const auto index = y*width+x;
                    if (probability >= 0.65 && hit_count >= 2) {
                        occupied[index] = 1;
                    } else if (hit_count > 0 && probability > 0.35) {
                        candidate[index] = 1;
                    } else if (probability <= 0.35) {
                        free[index] = 1;
                    }
                }
            }
        }
        std::ofstream grid(directory + "/grid.bin", std::ios::binary);
        for (std::size_t i = 0; i < occupied.size(); ++i) {
            const signed char value = occupied[i] ? 100 : (candidate[i] ? -1 : (free[i] ? 0 : -1));
            grid.write(reinterpret_cast<const char*>(&value), 1);
        }
        tree.write(directory + "/occupancy.ot");
        std::cout << "Replayed " << frame << " full scans; voxels=" << tree.size() << '\n';
        return 0;
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
