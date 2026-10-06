"""同场景地图及轨迹基准：首帧真值只用于评价坐标对齐，不用于建图。"""
import argparse
import json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation
from mini_nav_fastlivo.geometry import transform

parser = argparse.ArgumentParser()
parser.add_argument('capture', type=Path)
parser.add_argument('result', type=Path)
args = parser.parse_args()
capture = args.capture
result = json.loads(args.result.read_text())
measurement = json.loads((capture/'measurement.json').read_text())
trace = np.load(capture/'trace.npz')
session = Path(measurement['snapshots'][-1]['status']['recording_session'])
metadata = json.loads((session/'session.json').read_text())
anchor = np.asarray(metadata['map_world'])
imu_base = np.asarray(metadata['imu_base'])
normal, direction, offset, limits = measurement['line']
normal, direction = np.asarray(normal), np.asarray(direction)


def truth_at(stamp):
    index = np.argmin(abs(trace['truth'][:, 0]-stamp))
    if abs(trace['truth'][index, 0]-stamp) > 100_000_000:
        raise ValueError('Truth timestamp mismatch')
    return trace['truth'][index, 1:].reshape(4, 4)


first = trace['poses'][0]
map_gazebo = anchor @ first[1:].reshape(4, 4) @ imu_base @ np.linalg.inv(truth_at(first[0]))
slam_first = trace['slam_poses'][0]
slam_gazebo = slam_first[1:].reshape(4, 4) @ np.linalg.inv(truth_at(slam_first[0]))
map_slam = map_gazebo @ np.linalg.inv(slam_gazebo)
yaw = -1.5708
rotation = np.array([[np.cos(yaw), -np.sin(yaw)], [np.sin(yaw), np.cos(yaw)]])
vertices = np.array([[-400,230.94],[0,461.88],[400,230.94],[400,-230.94],
                     [0,-461.88],[-400,-230.94]])*.0254*.25
vertices = vertices @ rotation.T
vertices = vertices @ map_gazebo[:2, :2].T + map_gazebo[:2, 3]
edges = []
for a, b in zip(vertices, np.roll(vertices, -1, axis=0)):
    tangent = (b-a)/np.linalg.norm(b-a)
    n = np.array([-tangent[1], tangent[0]])
    if n@normal < 0:
        n = -n
    c = float(n@a)
    edges.append((np.linalg.norm(n-normal)+abs(c-offset), n, c))
_, true_normal, true_offset = min(edges, key=lambda edge: edge[0])


def points_of(data, resolution, origin, alignment):
    y, x = np.nonzero(data == 100)
    xyz = np.column_stack([(x+.5)*resolution+origin[0],
                           (y+.5)*resolution+origin[1], np.zeros(len(x))])
    return transform(xyz, alignment)[:, :2]


def stats(data, resolution, origin, alignment):
    xy = points_of(data, resolution, origin, alignment)
    perpendicular, along = xy@normal-offset, xy@direction
    chosen = (abs(perpendicular)<.4)&(along>limits[0])&(along<limits[1])
    if not chosen.any():
        raise ValueError('Common wall missing from map')
    bins = np.floor((along[chosen]-limits[0])/.1).astype(int)
    spans = [np.ptp(perpendicular[chosen][bins==i])+resolution for i in np.unique(bins)]
    inward = xy[chosen]@true_normal-true_offset
    half_cell = resolution*np.abs(alignment[:2, :2].T@true_normal).sum()/2
    return dict(median_width_m=float(np.median(spans)),
        p90_width_m=float(np.quantile(spans,.9)),
        inward_center_p95_m=float(np.quantile(inward,.95)),
        inward_area_p95_m=float(np.quantile(inward,.95)+half_cell),
        occupied_wall_cells=int(chosen.sum()))


images = []
for name in ['stationary_20s', 'final']:
    saved = np.load(capture/(name+'.npz'))
    images.append(('online_'+name, saved['grid'], float(saved['resolution']), saved['origin'], np.eye(4)))
for name, bundle in result['bundles'].items():
    saved = np.load(Path(bundle)/'observations.npz')
    data = np.where(saved['occupied'],100,np.where(saved['free'],0,-1)).astype(np.int8)
    images.append((name,data,metadata['resolution'],[-metadata['map_size']/2]*2,np.eye(4)))
for name in ['stationary_20s', 'final']:
    saved = np.load(capture/(name+'_slam.npz'))
    images.append(('slam_'+name,saved['grid'],float(saved['resolution']),saved['origin'],map_slam))
comparison = {name: stats(data,resolution,origin,alignment)
              for name,data,resolution,origin,alignment in images}
comparison['reference'] = dict(normal=true_normal.tolist(), offset=true_offset,
    map_gazebo=map_gazebo.tolist(),map_slam=map_slam.tolist(),
    gauge='first pose paired with independent Gazebo truth; no per-map re-fitting',
    limitation='Occupancy-band width is not physical wall thickness; input sensors differ')
trajectories = np.load(Path(result['work'])/'trajectories.npz')
for name in ['original', 'optimized']:
    estimates = anchor @ trajectories[name] @ imu_base
    positions = np.asarray([(map_gazebo@truth_at(stamp))[:2,3] for stamp in trajectories['stamps']])
    errors = np.linalg.norm(estimates[:,:2,3]-positions, axis=1)
    comparison['trajectory_'+name] = dict(rmse_xy_m=float(np.sqrt(np.mean(errors**2))),
        p95_xy_m=float(np.quantile(errors,.95)), final_xy_m=float(errors[-1]))
(capture/'comparison.json').write_text(json.dumps(comparison,indent=2))
fig, axes = plt.subplots(2,3,figsize=(15,9),sharex=True,sharey=True)
for axis,(name,data,resolution,origin,alignment) in zip(axes.ravel(),images):
    x,y = np.meshgrid(np.arange(data.shape[1]+1)*resolution+origin[0],
                      np.arange(data.shape[0]+1)*resolution+origin[1])
    points = transform(np.column_stack([x.ravel(),y.ravel(),np.zeros(x.size)]),alignment)
    display = np.where(data==100,0.,np.where(data==0,1.,.65))
    axis.pcolormesh(points[:,0].reshape(x.shape),points[:,1].reshape(y.shape),display,
                    cmap='gray',vmin=0,vmax=1,rasterized=True)
    metric=comparison[name]
    axis.set_title(f"{name}\nwall band {metric['median_width_m']*100:.2f} cm; inward P95 {metric['inward_center_p95_m']*100:.2f} cm",fontsize=9)
    axis.set_xlim(-2,2);axis.set_ylim(-1,2);axis.set_aspect('equal')
fig.tight_layout();fig.savefig(capture/'comparison.png',dpi=160)
print(json.dumps(comparison,indent=2))
