#!/usr/bin/env python3.8

import rospy
from dwa_util import *
# from sklearn.neighbors import KDTree
import pickle
from nav_msgs.msg import OccupancyGrid
import numpy as np

class CostMap:
    def __init__(self, robot_radius: float = 0.5, a: float = 1.5, b: float = 1.8):
        """Initialize the CostMap class."""
        self.util = MapUtil()
        self.grid_map = self.util.get_map()
        rospy.loginfo("object init done")

    def store_raw_map(self, name_of_file: str):
        """Store the raw map in a file."""
        with open(name_of_file, 'wb') as f:
            pickle.dump(self.grid_map, f)

if __name__=="__main__":
    node_name="raw_cost_map"
    rospy.init_node(node_name)
    rospy.loginfo("raw_cost_map begin")

    folder_path = "/home/nvidia/tcpb_ws/src/nav/scripts/DWA/"
    # map_path=rospy.get_param(node_name+"/map_path")
    file_name = "CostMap/"+"lg205back"
    cost_map = CostMap(robot_radius=0.3,a=2.5,b=1.8)
    rospy.loginfo("CostMap object done")
    cost_map.store_raw_map(folder_path+file_name)
    rospy.loginfo("cost map saved")

    with open(folder_path+file_name,'rb') as f:
        my_cost_map = pickle.load(f)

    rate = rospy.Rate(5)

    pub = rospy.Publisher("/modified_map", OccupancyGrid, queue_size=10)

    util = MapUtil()

    occupancy_grid = OccupancyGrid()
    occupancy_grid.header.frame_id = "map"
    occupancy_grid.info = util.get_map_info()
    occupancy_grid.data = my_cost_map

    rospy.loginfo("enter loop")

    while not rospy.is_shutdown():
        occupancy_grid.header.stamp = rospy.Time.now()
        pub.publish(occupancy_grid)

        rate.sleep()