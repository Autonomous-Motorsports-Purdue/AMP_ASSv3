from sensor_msgs.msg import PointCloud2, PointField
import ctypes
import struct
from sensor_msgs.msg import PointCloud2, PointField
import math
import sensor_msgs_py.point_cloud2 as pc
import rclpy
import numpy as np
from rclpy.node import Node

import cv2
from geometry_msgs.msg import Twist
import scipy.ndimage

from std_msgs.msg import Float32MultiArray, MultiArrayDimension

class Flatten(Node):

    def __init__(self):
        """
        Gets params and sets up subscribes/publishes to the required nodes.
        """
        super().__init__('flatten')

        # read costmap values, calculate next goal
        self.create_subscription(PointCloud2, '/nonground',
                                 self.callback, 2)

        self.costmap_pub = self.create_publisher(
            Float32MultiArray, '/costmapf', 3)
            
        self.get_logger().info(f"flattening init \n")
        
    def callback(self, ros_point_cloud):
        xyz = np.array([[0,0,0]])
        rgb = np.array([[0,0,0]])
        gen = pc.read_points(ros_point_cloud, skip_nans=True)
        int_data = list(gen)
        
        x_min = -5
        x_max = 5
        y_min = -5
        y_max = 5
        res = 0.05

        for x in int_data:
            test = x[3] 
            # cast float32 to int so that bitwise operations are possible
            s = struct.pack('>f' ,test)
            i = struct.unpack('>l',s)[0]
            # you can get back the float value by the inverse operations
            pack = ctypes.c_uint32(i).value
            r = (pack & 0x00FF0000)>> 16
            g = (pack & 0x0000FF00)>> 8
            b = (pack & 0x000000FF)
            # prints r,g,b values in the 0-255 range
                        # x,y,z can be retrieved from the x[0],x[1],x[2]
            xyz = np.append(xyz,[[x[0],x[1],x[2]]], axis = 0)
            # rgb = np.append(rgb,[[r,g,b]], axis = 0)
            
        in_range = (xyz[:, 0] < x_max) & (xyz[:, 0] > x_min) & (xyz[:, 1] < y_max) & (xyz[:, 1] > y_min)
        
        # print('inrange', in_range.shape)
        # print('xyz', xyz.shape)
        xyz_pixels = ((xyz[in_range, 0:2] - np.array([x_min, y_min])) / res).astype(int)
        heat = np.zeros((int((x_max-x_min)/res), int((y_max - y_min)/res)), dtype=int)
        heat[xyz_pixels[:, 0], xyz_pixels[:, 1]] = 230
        
        tosend = heat.flatten().tolist()
        tosend = [float(elem) for elem in tosend]
        costmap = Float32MultiArray()
        costmap.data = tosend

        costmap.layout.dim = [MultiArrayDimension(size=heat.shape[0]), MultiArrayDimension(size=heat.shape[1])]
        # print('sending this', costmap)
        
        self.get_logger().info(f"flattening publish \n")
        self.costmap_pub.publish(costmap)
        
        if False:
            cv2.imshow('heat', heat.astype(np.uint8))
            cv2.waitKey(1)

def main():
    rclpy.init(args=None)
    subscriber = Flatten()
    rclpy.spin(subscriber)
    subscriber.destroy_node()
    rclpy.shutdown()
    

if __name__ == '__main__':
    main()
