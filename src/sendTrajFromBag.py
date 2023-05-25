#! /usr/bin/env python3
import sys
import yaml
import rospy
import rosbag

from trajectory_msgs.msg import JointTrajectory

def main():

    rospy.init_node("traj_bag_publisher", anonymous=True)
    rospy.Rate(1)

    traj_pub = rospy.Publisher("/reference_trajectory", JointTrajectory, queue_size=1)

    fname = "/home/nick/catkin_ws/src/robust_fast_navigation/bags/slightCurveWorld10.bag"
    bag = rosbag.Bag(fname)

    traj_msg = None
    for topic, msg, t in bag.read_messages():

        traj_msg = msg
        # print(msg)
        # traj_pub.publish(msg)

    rospy.sleep(5)
    while not rospy.is_shutdown():

        traj_msg.header.stamp = rospy.Time.now()
        traj_pub.publish(traj_msg)
        sys.exit()


if __name__ == "__main__":
    main()
