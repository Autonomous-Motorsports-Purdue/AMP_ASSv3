import rclpy
import numpy as np
from rclpy.node import Node
from nav2_msgs.msg import Costmap
from nav_msgs.msg import Odometry
import cv2
from geometry_msgs.msg import Quaternion, PoseStamped, PoseWithCovarianceStamped
import scipy.ndimage
from transforms3d.euler import quat2euler

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

    initialKartYaw = 0
    currentKartYaw = 0
    currentKartQuat = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
    parabola_numer = 0
    parabola_denom = 0
    show_costmaps = False

    goal_x = 0
    goal_y = 0
    goal_z = 0

    def __init__(self):
        super().__init__('CostMap_Subscriber')

        goal_update_freq = self.get_parameter("/goal_update_freq")
        self.parabola_numer = self.get_parameter("/parabola_numer")
        self.parabola_denom = self.get_parameter("/parabola_denom")
        self.show_costmaps = self.get_parameter("/show_costmaps")

        # read costmap values, calculate next goal
        self.create_subscription(Costmap, '/local_costmap/costmap_raw',
                                 self.costmap_callback, 10)
        # update currentKartYaw
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        # set intialKartYaw
        self.create_subscription(PoseWithCovarianceStamped, '/initial',
                                 self.inital_pose_callback, 10)
        # Publish new goal pose for nav2 with timer
        self.goal_pose_publisher = self.create_publisher(
            PoseStamped, '/goal_pose', 10)
        self.goal_timer = self.create_timer(goal_update_freq,
                                            self.goal_pose_callback)

    def costmap_callback(self, msg):
        # Initialize costmap
        costmap = msg.data
        costmapWidth = msg.metadata.size_y
        costmapHeight = msg.metadata.size_x
        costmapResolution = msg.metadata.resolution
        xOffset = msg.metadata.origin.position.x
        yOffset = msg.metadata.origin.position.y
        costmapArray = np.zeros((costmapHeight, costmapWidth), dtype=np.uint8)
        costmapIndex = 0
        for i in range(costmapHeight):
            for j in range(costmapWidth):
                costmapArray[i][j] = COST_IMAGE_MAPPINGS.get(
                    costmap[costmapIndex], costmap[costmapIndex])
                costmapIndex += 1

        # threshold for costmap values that meet our minimum (atm, just under inflation buffer)
        threshCostmap = cv2.threshold(costmapArray, 100, 255,
                                      cv2.THRESH_BINARY_INV)[1]
        # add border around image to disincentivize picking values that are far from the kart
        threshCostmap = cv2.copyMakeBorder(threshCostmap, 1, 1, 1, 1,
                                           cv2.BORDER_CONSTANT, (0))

        # apply parabola mask to image
        mask = np.zeros((costmapHeight, costmapWidth))
        mask_x_values = np.linspace(-1, 1, costmapWidth * 2)
        mask_y_values = (self.parabola_numer / self.parabola_denom) * (
            (mask_x_values)**2)
        mask_x_indices = np.floor(
            ((mask_x_values + 1) / 2) * (costmapWidth - 1)).astype(int)
        mask_y_indices = np.floor(
            ((mask_y_values + 1) / 2) * (costmapHeight - 1)).astype(int)
        mask[mask_y_indices, mask_x_indices] = 1
        for value in zip(mask_y_indices, mask_x_indices):
            mask[0:value[0], value[1]] = 1
        mask = scipy.ndimage.rotate(
            mask,
            angle=np.rad2deg(-1 *
                             (self.currentKartYaw - self.initialKartYaw)) - 90,
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

        # scale points to real world distance
        max_loc_costmap_origin = max_loc
        max_loc_costmap_origin_scaled = tuple(
            cord * costmapResolution for cord in max_loc_costmap_origin)

        # move points to current offset of local map from global origin
        max_loc_costmap_origin_offset = (max_loc_costmap_origin_scaled[0] +
                                         xOffset,
                                         max_loc_costmap_origin_scaled[1] +
                                         yOffset)

        # log values
        self.get_logger().info(f"{[centx, centy]}")
        self.get_logger().info(f"{max_loc_costmap_origin_scaled}")
        self.get_logger().info(f"{max_loc_costmap_origin_offset}")
        self.get_logger().info(f"{msg.metadata.origin}")

        self.goal_x = max_loc_costmap_origin_offset[0]
        self.goal_y = max_loc_costmap_origin_offset[1]

        # display costmaps in GUI
        if self.show_costmaps:
            # add green point for current goal
            result = costmapArray.copy()
            result = cv2.cvtColor(result, cv2.COLOR_GRAY2BGR)
            centx = max_loc[0]
            centy = max_loc[1]
            result = cv2.circle(result, (centx, centy),
                                5, (0, 100, 0),
                                thickness=cv2.FILLED)

            # HACK: flip images so they are oriented correctly
            distImgDisplay = cv2.flip(distImgNormalized, 1)
            threshCostmapDisplay = cv2.flip(threshCostmap, 1)
            result = cv2.flip(result, 1)

            cv2.imshow('costmap', result)
            cv2.imshow('costmapThresh', threshCostmapDisplay)
            cv2.imshow('costampDist', distImgDisplay)
            cv2.waitKey(1)

        # NOTE: Rviz rotation flipped,
        # NOTE: Resolution is the scale of the points (meters/cell)
        # TODO: FMM planner, scipy

    def odom_callback(self, msg):
        orientationQuat = [
            msg.pose.pose.orientation.w, msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y, msg.pose.pose.orientation.z
        ]
        orientationEulers = quat2euler(orientationQuat)
        self.currentKartYaw = orientationEulers[2]

    def inital_pose_callback(self, msg):
        initial_pose_quat = [
            msg.pose.pose.orientation.w, msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y, msg.pose.pose.orientation.z
        ]
        initial_pose_euler = quat2euler(initial_pose_quat)
        self.initialKartYaw = initial_pose_euler[2]

    def goal_pose_callback(self):
        msg = PoseStamped()
        msg.pose.position.x = float(self.goal_x)
        msg.pose.position.y = float(self.goal_y)
        msg.pose.orientation = self.currentKartQuat
        self.goal_pose_publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    subscriber = CostMapSubscriber()

    rclpy.spin(subscriber)

    subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
