"""Save the latest /map as a PNG (cropped to known area) for inspection."""
import sys
import numpy as np
import rclpy
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from nav_msgs.msg import OccupancyGrid
from PIL import Image

rclpy.init()
n = rclpy.create_node('grab_map')
got = {}
qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
n.create_subscription(OccupancyGrid, '/map', lambda m: got.setdefault('m', m), qos)
while 'm' not in got:
    rclpy.spin_once(n, timeout_sec=1.0)
m = got['m']
a = np.array(m.data, dtype=np.int16).reshape(m.info.height, m.info.width)
img = np.full(a.shape, 205, np.uint8)
known = a >= 0
img[known] = (255 - a[known] * 255 // 100).astype(np.uint8)
ys, xs = np.where(known)
pad = 10
img = img[max(ys.min()-pad, 0):ys.max()+pad, max(xs.min()-pad, 0):xs.max()+pad]
Image.fromarray(img[::-1]).save(sys.argv[1])
print('saved', sys.argv[1], img.shape, 'res', m.info.resolution)
