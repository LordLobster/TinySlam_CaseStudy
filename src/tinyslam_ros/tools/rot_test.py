"""Diagnostics: stationary drift then in-place rotations; prints slam/odom heading error vs ground truth."""
import math, re, subprocess, sys, time
import rclpy
from rclpy.parameter import Parameter
from geometry_msgs.msg import TwistStamped, PoseStamped
from nav_msgs.msg import Odometry

def yaw(q): return math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))
def wrap(a): return math.atan2(math.sin(a), math.cos(a))
def truth():
    out = subprocess.run(['gz', 'model', '-m', 'ICTE4001_MRobot', '-p'], capture_output=True, text=True, timeout=10).stdout
    v = [list(map(float, l)) for l in re.findall(r'\[\s*(-?[\d.]+)\s+(-?[\d.]+)\s+(-?[\d.]+)\]', out)]
    return v[0][0], v[0][1], v[1][2]

rclpy.init()
n = rclpy.create_node('rot_test', parameter_overrides=[Parameter('use_sim_time', value=True)])
st = {}
n.create_subscription(Odometry, '/diffdrive_controller/odom', lambda m: st.__setitem__('odom', m.pose.pose), 10)
n.create_subscription(PoseStamped, '/tinyslam/pose', lambda m: st.__setitem__('slam', m.pose), 10)
pub = n.create_publisher(TwistStamped, '/cmd_vel', 10)

def spin_for(sec, wz=0.0, vx=0.0):
    end = time.monotonic() + sec
    while time.monotonic() < end:
        c = TwistStamped(); c.header.stamp = n.get_clock().now().to_msg(); c.header.frame_id = 'base_footprint'
        c.twist.angular.z = wz; c.twist.linear.x = vx
        pub.publish(c)
        rclpy.spin_once(n, timeout_sec=0.05)

def report(tag):
    spin_for(1.5)
    gx, gy, gth = truth()
    s, o = st['slam'], st['odom']
    es = math.degrees(wrap(yaw(s.orientation) - gth)); eo = math.degrees(wrap(yaw(o.orientation) - gth))
    ps = math.hypot(s.position.x - gx, s.position.y - gy)
    print(f'{tag:>22}: truth yaw {math.degrees(gth):7.1f}  slam yaw err {es:+6.2f} deg pos err {ps:.3f} m | odom yaw err {eo:+6.2f}', flush=True)

while 'slam' not in st or 'odom' not in st:
    rclpy.spin_once(n, timeout_sec=0.5)
report('start')
spin_for(15); report('stationary 15s')
for wz in [float(a) for a in (sys.argv[1:] or ['0.3', '0.6'])]:
    spin_for(2 * math.pi / wz, wz=wz); report(f'360 deg @ {wz} rad/s')
