#!/usr/bin/env python3
import rospy
import numpy as np
import actionlib
from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from tf.transformations import quaternion_from_euler

class FrontierExplorer:
    def __init__(self):
        rospy.init_node('frontier_explorer')
        
        # Subscribe to the occupancy grid (map)
        self.map_sub = rospy.Subscriber('/map', OccupancyGrid, self.map_callback)
        # Optionally subscribe to odometry if you need the robot's current pose
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        
        # Action client for move_base
        rospy.loginfo("Waiting for move_base action server...")
        self.move_base_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.move_base_client.wait_for_server()
        rospy.loginfo("Connected to move_base!")
        
        self.map_data = None
        self.map_info = None
        self.robot_pose = None

    def map_callback(self, msg):
        # Convert the occupancy grid data into a 2D NumPy array
        self.map_info = msg.info
        self.map_data = np.array(msg.data).reshape((msg.info.height, msg.info.width))
        rospy.loginfo("Map updated: size %d x %d", msg.info.width, msg.info.height)

    def odom_callback(self, msg):
        # For this simple example, we can store the robot's position if needed
        self.robot_pose = msg.pose.pose

    def detect_frontiers(self):
        """
        Detect frontiers in the map.
        A frontier is defined as a free cell (value 0) that borders an unknown cell (value -1).
        Returns a list of frontier cell indices.
        """
        if self.map_data is None:
            return []

        frontiers = []
        grid = self.map_data
        height, width = grid.shape

        # Iterate through cells, skipping the borders for simplicity
        for i in range(1, height - 1):
            for j in range(1, width - 1):
                if grid[i, j] == 0:  # free cell
                    # Check the 8-neighborhood for unknown cells (-1)
                    neighborhood = grid[i - 1:i + 2, j - 1:j + 2]
                    if np.any(neighborhood == -1):
                        frontiers.append((i, j))
        rospy.loginfo("Detected %d frontier cells", len(frontiers))
        return frontiers

    def choose_goal(self, frontiers):
        """
        For simplicity, choose the frontier cell closest to the center of the map.
        You can improve this by considering the robot's current pose, distance, etc.
        """
        if not frontiers or self.map_info is None:
            return None

        center = np.array([self.map_info.height / 2.0, self.map_info.width / 2.0])
        distances = [np.linalg.norm(np.array(pt) - center) for pt in frontiers]
        min_index = np.argmin(distances)
        return frontiers[min_index]

    def cell_to_world(self, cell):
        """
        Convert a grid cell (i, j) to world coordinates using map resolution and origin.
        """
        i, j = cell
        resolution = self.map_info.resolution
        origin = self.map_info.origin.position
        # Note: Adjust the conversion based on how your map is defined.
        x = origin.x + j * resolution + resolution / 2.0
        # In many maps, row 0 is at the top, so you might need to flip the i index.
        y = origin.y + (self.map_info.height - i) * resolution - resolution / 2.0
        return x, y

    def send_goal(self, x, y):
        """
        Send a navigation goal to move_base.
        """
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        # Set a default orientation (facing forward)
        q = quaternion_from_euler(0, 0, 0)
        goal.target_pose.pose.orientation.x = q[0]
        goal.target_pose.pose.orientation.y = q[1]
        goal.target_pose.pose.orientation.z = q[2]
        goal.target_pose.pose.orientation.w = q[3]
        rospy.loginfo("Sending goal: x = %.2f, y = %.2f", x, y)
        self.move_base_client.send_goal(goal)
        # Optionally, wait for the result:
        finished_within_time = self.move_base_client.wait_for_result(rospy.Duration(60))
        if not finished_within_time:
            rospy.logwarn("Timed out achieving goal")
            self.move_base_client.cancel_goal()
        else:
            state = self.move_base_client.get_state()
            if state == 3:
                rospy.loginfo("Goal reached successfully")
            else:
                rospy.logwarn("Failed to reach goal, state: %d", state)

    def run(self):
        rate = rospy.Rate(0.1)  # Run the exploration loop at 0.1 Hz (every 10 seconds)
        while not rospy.is_shutdown():
            if self.map_data is not None:
                frontiers = self.detect_frontiers()
                if frontiers:
                    goal_cell = self.choose_goal(frontiers)
                    if goal_cell:
                        x, y = self.cell_to_world(goal_cell)
                        self.send_goal(x, y)
                    else:
                        rospy.loginfo("No suitable frontier cell found.")
                else:
                    rospy.loginfo("No frontiers detected. Waiting for more map updates...")
            else:
                rospy.loginfo("Waiting for map data...")
            rate.sleep()

if __name__ == '__main__':
    try:
        explorer = FrontierExplorer()
        explorer.run()
    except rospy.ROSInterruptException:
        pass
