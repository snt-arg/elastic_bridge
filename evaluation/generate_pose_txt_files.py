#!/usr/bin/env python3
import sys
import rospy
from geometry_msgs.msg import PoseStamped
import yaml

# Load the configurations
config_file = open("config.yaml", "r")
config = yaml.load(config_file, Loader=yaml.FullLoader)

files_path = config["results_dir"]
slam_method = config["slam_method"]
dataset_seq = config["dataset_seq"]
slam_pose_topic = config["ros_topics"]["camera_pose"]

if len(sys.argv) > 1:
    # use that as identifier
    dataset_seq += sys.argv[1]

# Creating a txt file that will contain poses
print("Creating txt files for adding SLAM poses ...")
slam_pose_file = open(
    f"{files_path}/slam_pose_{slam_method}_{dataset_seq}.txt", "w+")
slam_pose_file.write("#timestamp tx ty tz qx qy qz qw\n")
slam_pose_file.flush()


def slamPoseCallback(slam_pose_msg):
    # Calculate different variables
    time = slam_pose_msg.header.stamp.to_sec()
    odom_x = slam_pose_msg.pose.position.x
    odom_y = slam_pose_msg.pose.position.y
    odom_z = slam_pose_msg.pose.position.z
    odom_rx = slam_pose_msg.pose.orientation.x
    odom_ry = slam_pose_msg.pose.orientation.y
    odom_rz = slam_pose_msg.pose.orientation.z
    odom_rw = slam_pose_msg.pose.orientation.w
    # Write to the SLAM file
    slam_pose_file.write(
        str(time) + " " + str(odom_x) + " " + str(odom_y) + " "
        + str(odom_z) + " " + str(odom_rx) + " " + str(odom_ry) + " "
        + str(odom_rz) + " " + str(odom_rw) + "\n"
    )
    slam_pose_file.flush()


def subscribers():
    rospy.init_node("text_file_generator", anonymous=True)
    # Subscriber to the SLAM topic
    rospy.Subscriber(slam_pose_topic, PoseStamped, slamPoseCallback)
    rospy.spin()


if __name__ == "__main__":
    subscribers()