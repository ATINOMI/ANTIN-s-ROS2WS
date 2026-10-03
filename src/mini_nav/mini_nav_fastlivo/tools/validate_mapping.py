#!/usr/bin/env python3
"""仅在224域独立Gazebo中验证两入口；不会连接用户运行域。"""
import json, os, pathlib, signal, subprocess, time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
if os.environ.get('ROS_DOMAIN_ID') != '224':
    raise RuntimeError('Isolated domain 224 required')
root=pathlib.Path('/home/a/ros2_ws')
evidence=root/'src/mini_nav/logs/26-10-3/fastlivo_navigation_evidence'
evidence.mkdir(exist_ok=True)
rclpy.init()
node=Node('mapping_acceptance')
state={}
poses=[]
commands=[]
latched=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
node.create_subscription(String,'/fastlivo/mapping_status',lambda m: state.update(json.loads(m.data)),latched)
node.create_subscription(Odometry,'/odom',lambda m: poses.append([m.pose.pose.position.x,m.pose.pose.position.y]),10)
node.create_subscription(TwistStamped,'/cmd_vel',lambda m: commands.append([m.twist.linear.x,m.twist.angular.z]),10)
def spin(seconds):
    end=time.monotonic()+seconds
    while time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.01)
def wait(test,seconds=50):
    end=time.monotonic()+seconds
    while time.monotonic()<end:
        rclpy.spin_once(node,timeout_sec=.02)
        if test():return
    raise RuntimeError('Timeout: '+json.dumps(state))
spin(.4)
assert node.count_publishers('/clock')==0
log=(evidence/'mapping_launch.log').open('w')
process=subprocess.Popen(['ros2','launch','mini_nav_bringup','fastlivo_mapping.launch.py',
 'ros_domain_id:=224','gz_partition:=mini_nav_fastlivo_acceptance','gui:=false','rviz:=false',
 'teleop:=false','external_control:=true','output_dir:=/home/a/ros2_ws/maps/fastlivo2_acceptance'],
 stdout=log,stderr=log,start_new_session=True)
timer=None
try:
    wait(lambda: state.get('scan_observations',0)>10 and state.get('geometry_points',0)>1000)
    assert state['valid'],state
    raw=node.create_publisher(TwistStamped,'/mini_nav/cmd_vel_raw',1)
    lease=node.create_publisher(Bool,'/mini_nav/task_active',1)
    speed=[0.]
    def tick():
        m=TwistStamped();m.header.stamp=node.get_clock().now().to_msg()
        # Probe uses ROS wall time, but guard uses simulation time. Copy odom stamp below.
        m.header.stamp=stamp[0]
        m.header.frame_id='base_footprint';m.twist.linear.x=speed[0]
        raw.publish(m);lease.publish(Bool(data=speed[0]!=0))
    stamp=[None]
    node.create_subscription(Odometry,'/odom',lambda m: stamp.__setitem__(0,m.header.stamp),10)
    wait(lambda: stamp[0] is not None)
    timer=node.create_timer(.05,tick)
    start=poses[-1]
    speed[0]=.10;spin(8);speed[0]=0.;spin(2)
    service=node.create_client(Trigger,'/fastlivo/save_nav_map')
    wait(service.service_is_ready,5)
    future=service.call_async(Trigger.Request())
    wait(future.done,10)
    answer=future.result()
    assert answer.success,answer.message
    result={'map_bundle':answer.message,'status':state,'start':start,'end':poses[-1],
            'cmd_vel_publishers':node.count_publishers('/cmd_vel'),
            'moved':any(abs(v[0])>.01 for v in commands)}
    assert result['moved']
    assert result['cmd_vel_publishers']==1
    (evidence/'mapping_result.json').write_text(json.dumps(result,indent=2))
    print(json.dumps(result),flush=True)
finally:
    if timer:timer.cancel()
    if process.poll() is None:
        os.killpg(process.pid,signal.SIGINT)
        try:process.wait(timeout=12)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid,signal.SIGTERM);process.wait(timeout=8)
    log.close();node.destroy_node();rclpy.shutdown()
