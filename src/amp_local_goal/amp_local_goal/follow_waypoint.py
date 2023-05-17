import math
import rclpy
import numpy as np
from rclpy.node import Node

import cv2
from geometry_msgs.msg import Twist
import scipy.ndimage
from std_msgs.msg import Float32MultiArray

# Costmap values (hardcoded in ROS)
LETHAL_OBSTACLE = 254
NO_INFO = 255
INFLATION_BUFFER = 253
MEDIUM_COST = 128
FREE_SPACE = 0

# Remapping costmap values to custom brightness values in costmap image
COST_IMAGE_MAPPINGS = {
    LETHAL_OBSTACLE: 255,
    NO_INFO: 0,
    INFLATION_BUFFER: 128,
    MEDIUM_COST: 192,
    FREE_SPACE: 0
}


class CostMapSubscriber(Node):

    def __init__(self):
        """
        Gets params and sets up subscribes/publishes to the required nodes.
        """
        super().__init__('find_local_goal')

        self.declare_parameter('parabola_ratio', 1)
        self.declare_parameter('show_costmaps', True)
        self.declare_parameter('costmap_threshold', 100)

        self.parabola_ratio = self.get_parameter("parabola_ratio").value
        self.show_costmaps = self.get_parameter("show_costmaps").value
        self.costmap_threshold = self.get_parameter("costmap_threshold").value

        # read costmap values, calculate next goal
        self.create_subscription(Float32MultiArray, '/costmapf',
                                 self.costmap_callback, 10)

        # Publish new goal pose for nav2 with timer
        self.goal_vel_publisher = self.create_publisher(
            Twist, '/cmd_vel', 10)

    def costmap_callback(self, msg):
       
        # Initialize costmap
        costmap = msg.data
        costmapWidth = msg.layout.dim[0].size
        costmapHeight = msg.layout.dim[1].size
        
        costmapArray = np.zeros((costmapHeight, costmapWidth), dtype=np.uint8)

        print(type(costmap))

        costmapIndex = 0
        for i in range(costmapHeight):
            for j in range(costmapWidth):
                costmapArray[i][j] = costmap[costmapIndex]
                costmapIndex += 1

        # threshold for costmap values that meet our minimum (atm, just under inflation buffer)
        threshCostmap = cv2.threshold(costmapArray, self.costmap_threshold,
                                      255, cv2.THRESH_BINARY_INV)[1]
        # add border around image to disincentivize picking values that are far from the kart
        threshCostmap = cv2.copyMakeBorder(threshCostmap, 1, 1, 1, 1,
                                           cv2.BORDER_CONSTANT, (0))

        # apply parabola mask to image
        # TODO: Update this to use np.meshgrid instead of linspace
        mask = np.zeros((costmapHeight, costmapWidth))
        mask_x_values = np.linspace(-1, 1, costmapWidth * 2)
        mask_y_values = self.parabola_ratio * mask_x_values**2
        mask_x_indices = np.floor(
            ((mask_x_values + 1) / 2) * (costmapWidth - 1)).astype(int)
        mask_y_indices = np.floor(
            ((mask_y_values + 1) / 2) * (costmapHeight - 1)).astype(int)
        mask[mask_y_indices, mask_x_indices] = 1
        for value in zip(mask_y_indices, mask_x_indices):
            mask[0:value[0], value[1]] = 1

        threshCostmap = scipy.ndimage.rotate(
            threshCostmap,
            angle=-90,
            reshape=False,
            mode='nearest')
        mask = cv2.copyMakeBorder(mask, 1, 1, 1, 1, cv2.BORDER_CONSTANT, (0))
        maskedThershCostmap = (threshCostmap * mask).astype('uint8')

        # Compute Distance Transform
        distImg = cv2.distanceTransform(maskedThershCostmap, cv2.DIST_L2, 5)
        distImgNormalized = distImg.copy()
        cv2.normalize(distImg, distImgNormalized, 0, 1.0, cv2.NORM_MINMAX)

        # Find point furthest away from all obstacles
        max_loc = cv2.minMaxLoc(distImgNormalized)[3]

        self.get_logger().info(f"local goal coords: {max_loc[0]} {max_loc[1]}")
        
        delta_x = max_loc[0] - costmapWidth
        delta_y = max_loc[1] - costmapHeight
        angle = math.atan2(delta_y, delta_x) - math.pi / 2

        CENTER_POWER = 0.38
        MIN_POWER = 0.1

        # a 70 deg turn drops you 100% power

        power = 1 - math.degrees(abs(angle)) / 70
        if power < MIN_POWER:
            power = MIN_POWER
        scaled_power = power * 0.5 + CENTER_POWER
        

        msg = Twist()
        msg.linear.x = scaled_power
        msg.angular.z = angle * 1
        self.goal_vel_publisher.publish(msg)

        # display costmaps in GUI
        if self.show_costmaps:
            self.get_logger().info(
                f"Plotted goal pixel coords: {[max_loc[0], max_loc[1]]}")

            # add green point for current goal
            result = costmapArray.copy()
            result = cv2.cvtColor(result, cv2.COLOR_GRAY2BGR)
            centx = max_loc[0]
            centy = max_loc[1]
            result = cv2.circle(result * 10, (centx, centy),
                                5, (0, 100, 0),
                                thickness=cv2.FILLED)

            # HACK: flip images so they are oriented correctly as they are shown in Rviz
            distImgDisplay = cv2.flip(distImgNormalized, 1)
            threshCostmapDisplay = cv2.flip(threshCostmap, 1)
            result = cv2.flip(result, 1)

            cv2.imshow('costmap', cv2.resize(result, (500, 500)))
            cv2.imshow('costmapThresh', cv2.resize(threshCostmapDisplay, (500, 500)))
            cv2.imshow('costampDist', cv2.resize(distImgDisplay, (500, 500)))
            cv2.waitKey(1)


        # NOTE: Rviz rotation flipped,
        # NOTE: Resolution is the scale of the points (meters/cell)


def main(args=None):
    rclpy.init(args=args)
    subscriber = CostMapSubscriber()
    rclpy.spin(subscriber)
    subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
