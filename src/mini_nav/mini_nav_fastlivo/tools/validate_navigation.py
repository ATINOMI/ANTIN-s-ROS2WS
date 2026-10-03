#!/usr/bin/env python3
"""独立重启导航：只读旧地图、人工初值、Action到达与最终零速。"""
import json,os,pathlib,signal,subprocess,time,hashlib,math
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile,DurabilityPolicy
from nav2_msgs.action import NavigateToPose,ComputePathToPose
from std_msgs.msg import String,Bool
from geometry_msgs.msg import PoseWithCovarianceStamped,TwistStamped
from nav_msgs.msg import Odometry,OccupancyGrid
from sensor_msgs.msg import LaserScan
from scipy.spatial.transform import Rotation
if os.environ.get('ROS_DOMAIN_ID')!='224':raise RuntimeError('Domain 224 required')
root=pathlib.Path('/home/a/ros2_ws');evidence=root/'src/mini_nav/logs/26-10-3/fastlivo_navigation_evidence'
bundle=pathlib.Path(json.loads((evidence/'mapping_result.json').read_text())['map_bundle'])
alignment=json.loads((bundle/'alignment.json').read_text())
before={p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in bundle.iterdir()}
rclpy.init();node=Node('navigation_acceptance',parameter_overrides=[Parameter('use_sim_time',value=True)])
state={};status=[];controllers=[];guards=[];scaninfo=[];local=[False];command=[0.,0.];odom=[]
latched=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
node.create_subscription(String,'/mini_nav/localization_state',lambda m:state.update(json.loads(m.data)),10)
node.create_subscription(String,'/mini_nav/controller_status',lambda m:controllers.append(m.data),latched)
node.create_subscription(String,'/mini_nav/velocity_guard_status',lambda m:guards.append(m.data),10)
node.create_subscription(LaserScan,'/scan',lambda m:scaninfo.append(min(m.ranges)),10)
node.create_subscription(String,'/mini_nav/navigation_status',lambda m:status.append(m.data),latched)
node.create_subscription(Bool,'/mini_nav/local_costmap_valid',lambda m:local.__setitem__(0,m.data),1)
node.create_subscription(TwistStamped,'/cmd_vel',lambda m:command.__setitem__(slice(None),[m.twist.linear.x,m.twist.angular.z]),10)
node.create_subscription(Odometry,'/odom',lambda m:odom.append([m.pose.pose.position.x,m.pose.pose.position.y]),10)
initial_pub=node.create_publisher(PoseWithCovarianceStamped,'/initialpose',10)
client=ActionClient(node,NavigateToPose,'/navigate_to_pose')
def spin(seconds):
 end=time.monotonic()+seconds
 while time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.01)
def wait(test,seconds=40):
 end=time.monotonic()+seconds
 while time.monotonic()<end:
  rclpy.spin_once(node,timeout_sec=.02)
  if test():return
 raise RuntimeError('Timeout '+json.dumps({'localization':state,'local_valid':local,'status':status[-12:],'command':command}))
spin(.3);assert node.count_publishers('/clock')==0
log=(evidence/'navigation_launch.log').open('w')
process=subprocess.Popen(['ros2','launch','mini_nav_bringup','fastlivo_navigation.launch.py',
 'ros_domain_id:=224','gz_partition:=mini_nav_fastlivo_acceptance','gui:=false','rviz:=false','x_pose:=-1.75','yaw:=0.15','map_bundle:='+str(bundle)],
 stdout=log,stderr=log,start_new_session=True)
result={}
try:
 wait(lambda:state.get('estimate_stamp_ns',0)>0)
 initial=PoseWithCovarianceStamped();initial.header.frame_id='map';initial.header.stamp=node.get_clock().now().to_msg()
 base=alignment['first_map_base'];initial.pose.pose.position.x=base[0][3]+.25;initial.pose.pose.position.y=base[1][3]
 q=Rotation.from_euler('z',.15).as_quat()
 initial.pose.pose.orientation.x,initial.pose.pose.orientation.y,initial.pose.pose.orientation.z,initial.pose.pose.orientation.w=map(float,q)
 initial_pub.publish(initial)
 wait(lambda:state.get('valid') and local[0],20)
 localized=dict(state)
 wait(client.server_is_ready,5)
 spin(.4)
 goal=NavigateToPose.Goal();goal.pose.header.frame_id='map';goal.pose.header.stamp=node.get_clock().now().to_msg()
 goal.pose.pose.position.x=base[0][3]+.55;goal.pose.pose.position.y=base[1][3];goal.pose.pose.orientation.w=1.
 planner=ActionClient(node,ComputePathToPose,'/compute_path_to_pose')
 wait(planner.server_is_ready,5)
 plan=ComputePathToPose.Goal();plan.goal=goal.pose
 planned=planner.send_goal_async(plan);wait(planned.done,5)
 planned_result=planned.result().get_result_async();wait(planned_result.done,5)
 print('planner probe',planned_result.result().result.error_code,planned_result.result().result.error_msg,flush=True)
 planner.destroy()
 future=client.send_goal_async(goal);wait(future.done,5);handle=future.result()
 assert handle.accepted, state
 outcome=handle.get_result_async();wait(outcome.done,35)
 response=outcome.result()
 spin(.5)
 after={p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in bundle.iterdir()}
 result={'localized':localized,'result_status':response.status,'error':response.result.error_msg,
  'final_command':command,'last_localization':state,'last_odom':odom[-1] if odom else None,
  'map_unchanged':before==after,'cmd_vel_publishers':node.count_publishers('/cmd_vel'),
  'map_publishers':node.count_publishers('/map'),'controller_reasons':list(dict.fromkeys(controllers)),'guard_reasons':list(dict.fromkeys(guards)),'navigation_status':status[-15:],'min_scan':min(scaninfo) if scaninfo else None,'mapping_nodes':[n for n in node.get_node_names() if 'store' in n or 'height_mapper' in n]}
 (evidence/'navigation_result.json').write_text(json.dumps(result,indent=2))
 result['different_spawn']={'x':-1.75,'yaw':.15}
 (evidence/'navigation_offset_result.json').write_text(json.dumps(result,indent=2))
 print(json.dumps(result),flush=True)
 assert response.status==4,result
 assert before==after and command==[0.,0.] and not result['mapping_nodes'],result
finally:
 if process.poll() is None:
  os.killpg(process.pid,signal.SIGINT)
  try:process.wait(timeout=12)
  except subprocess.TimeoutExpired:os.killpg(process.pid,signal.SIGTERM);process.wait(timeout=8)
 if not result:
  (evidence/'navigation_failure.json').write_text(json.dumps({'localization':state,'status':status[-20:],'local_valid':local,'command':command},indent=2))
 log.close();client.destroy();node.destroy_node();rclpy.shutdown()
