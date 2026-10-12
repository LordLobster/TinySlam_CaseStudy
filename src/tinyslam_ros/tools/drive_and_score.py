"""Drive a waypoint loop using odometry; at each waypoint log ground truth vs odom vs tinySLAM."""
import math, re, subprocess, sys, time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped, PoseStamped
from nav_msgs.msg import Odometry

WAYPOINTS = [(3.3, 0.0), (3.3, -4.5), (9.3, -4.5), (-6.7, -4.5), (3.3, -4.5), (3.3, 0.0), (0.0, 0.0)]
ROBOT = 'ICTE4001_MRobot'

def yaw(q): return math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))
def wrap(a): return math.atan2(math.sin(a), math.cos(a))

class Driver(Node):
    def __init__(self):
        super().__init__('drive_and_score', parameter_overrides=[rclpy.parameter.Parameter('use_sim_time', value=True)])
        self.pub = self.create_publisher(TwistStamped, '/cmd_vel', 10)
        self.odom = self.slam = self.gt = None
        self.create_subscription(Odometry, '/diffdrive_controller/odom', lambda m: setattr(self, 'odom', m.pose.pose), 10)
        self.create_subscription(PoseStamped, '/tinyslam/pose', lambda m: setattr(self, 'slam', m.pose), 10)
        self.wp = 0; self.errs = []; self.settle_until = None
        self.create_timer(0.05, self.step)
    def truth(self):
        out = subprocess.run(['gz', 'model', '-m', ROBOT, '-p'], capture_output=True, text=True, timeout=10).stdout
        v = [list(map(float, l)) for l in re.findall(r'\[\s*(-?[\d.]+)\s+(-?[\d.]+)\s+(-?[\d.]+)\]', out)]
        return v[0][0], v[0][1], v[1][2]
    def report(self, tag):
        o, s = self.odom, self.slam
        gx, gy, gth = self.truth()
        def err(p): return math.hypot(p.position.x-gx, p.position.y-gy), math.degrees(abs(wrap(yaw(p.orientation)-gth)))
        eo, es = err(o), err(s)
        self.errs.append((eo, es))
        print(f'{tag:>10}: truth=({gx:6.2f},{gy:6.2f},{math.degrees(gth):7.1f})  '
              f'odom err={eo[0]:.3f} m {eo[1]:5.2f} deg   slam err={es[0]:.3f} m {es[1]:5.2f} deg', flush=True)
    def step(self):
        if self.odom is None or self.slam is None:
            return
        cmd = TwistStamped(); cmd.header.stamp = self.get_clock().now().to_msg(); cmd.header.frame_id = 'base_footprint'
        if self.wp >= len(WAYPOINTS):
            self.pub.publish(cmd); raise SystemExit
        tx, ty = WAYPOINTS[self.wp]
        x, y, th = self.odom.position.x, self.odom.position.y, yaw(self.odom.orientation)
        dist = math.hypot(tx-x, ty-y); herr = wrap(math.atan2(ty-y, tx-x) - th)
        if dist < 0.05 or self.settle_until is not None:
            # Stop, let the robot settle and SLAM process a few scans, then score
            self.pub.publish(cmd)
            now = time.monotonic()
            if self.settle_until is None:
                self.settle_until = now + 1.5
            elif now >= self.settle_until:
                self.report(f'wp{self.wp} {WAYPOINTS[self.wp]}'); self.wp += 1; self.settle_until = None
            return
        if abs(herr) > 0.1:
            cmd.twist.angular.z = max(-0.6, min(0.6, 1.5*herr))
        else:
            cmd.twist.linear.x = min(0.6, 0.8*dist); cmd.twist.angular.z = 1.5*herr
        self.pub.publish(cmd)

rclpy.init()
n = Driver()
try:
    rclpy.spin(n)
except SystemExit:
    pass
es = [e[1][0] for e in n.errs]; eo = [e[0][0] for e in n.errs]
print(f'max position error: odom {max(eo):.3f} m, slam {max(es):.3f} m')
