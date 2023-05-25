#! /usr/bin/env python3
import sys
import rospy
import numpy as np
import matplotlib.pyplot as plt

from scipy import interpolate
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

odom = None
traj_pub = None

def odomcb(msg):
    global odom
    odom = msg

def sendLine():
    global traj_pub

    # t = np.array([0, .6, 1.2, 1.8, 2.4, 3.0, 3.6])
    # y = np.array([0, 0, 0, 0, 0, 0, 0])
    # x = np.array([0, .3, .6, .9, 1.2, 1.5, 1.8])

    x = np.array([0,0.464504820333041,0.955302366345312,1.34092900964067,1.28834355828221,1.07800175284838,0.569675723049957,-0.832602979842244,-1.69149868536372,-2.07712532865907,-1.91936897458370,-1.93689745836985,-2.21735319894829,-2.53286590709904,-2.60297984224365,-2.32252410166521,-1.63891323400526,-0.920245398773007,-0.359333917616126,0.376862401402279,0.902716914986854,1.53374233128834,2.05959684487292,2.39263803680982,2.56792287467134,2.58545135845749,2.63803680981595,2.69062226117441,2.93602103418054,2.60297984224365])
    y = np.array([0,0.192813321647678,0.508326029798424,1.03418054338300,1.61262050832603,1.96319018404908,1.96319018404908,1.91060473269062,1.52497808939527,0.946538124452236,0.385626643295356,-0.175284837861524,-0.911481156879930,-1.47239263803681,-2.05083260297984,-2.29623137598598,-2.24364592462752,-2.15600350569676,-2.08588957055215,-2.01577563540754,-2.13847502191060,-2.38387379491674,-2.48904469763365,-2.27870289219982,-1.68273444347064,-0.929009640666082,-0.0701139351446094,0.683610867659948,1.54250657318142,2.03330411919369])
    t = np.linspace(0,90., x.shape[0])

    theta = -np.pi/2
    R = np.array([[np.cos(theta), -np.sin(theta)],[np.sin(theta), np.cos(theta)]])
    print(R)
    for i in range(len(x)):
        [nX,nY] = np.matmul(R,np.array([x[i],y[i]]))
        x[i] = nX
        y[i] = nY

    # quat = odom.pose.pose.orientation
    # yaw = euler_from_quaternion((quat.x, quat.y, quat.z, quat.w))[0]
    # x = x*np.cos(yaw)
    # y = y*np.sin(yaw)

    # x += odom.pose.pose.position.x
    # y += odom.pose.pose.position.y

    cX = interpolate.CubicSpline(t, x, bc_type='clamped')
    cY = interpolate.CubicSpline(t, y, bc_type='clamped')

    vX = cX.derivative(1)
    vY = cY.derivative(1)

    aX = cY.derivative(2)
    aY = cY.derivative(2)

    totT = 0.0
    msg = JointTrajectory()
    # msg.header.frame_id = '/vicon/spot/spot'
    msg.header.frame_id = '/odom'
    msg.header.stamp = rospy.Time.now()

    while totT < t[-1]:

        msg.points.append(JointTrajectoryPoint())
        msg.points[-1].positions.append(cX(totT))
        msg.points[-1].positions.append(cY(totT))

        msg.points[-1].velocities.append(vX(totT))
        msg.points[-1].velocities.append(vY(totT))

        msg.points[-1].accelerations.append(aX(totT))
        msg.points[-1].accelerations.append(aY(totT))

        msg.points[-1].time_from_start = rospy.Duration(totT)
        totT += .1

    rospy.loginfo("********* SENDING LINE TRAJECTORY *********")
    traj_pub.publish(msg)


def sendCircle():
    
    R = 1
    T = 8

    quat = odom.pose.pose.orientation
    yaw = euler_from_quaternion((quat.x, quat.y, quat.z, quat.w))[0]


    # centerX = odom.pose.pose.position.x - R*np.cos(yaw)
    # centerY = odom.pose.pose.position.y - R*np.sin(yaw)

    # t = np.linspace(0,5,num=50)
    # theta = 2*np.pi * t/5
    # x = R*np.cos(-theta+yaw) + centerX
    # y = R*np.sin(-theta+yaw) + centerY

    t = np.linspace(0,T,num=50)
    theta = 2*np.pi * t/T
    x = R*np.cos(theta-np.pi/2)
    y = R*np.sin(theta-np.pi/2) + R
    

    cX = interpolate.CubicSpline(t, x, bc_type='clamped')
    cY = interpolate.CubicSpline(t, y, bc_type='clamped')

    vX = cX.derivative(1)
    vY = cY.derivative(1)

    aX = cX.derivative(2)
    aY = cY.derivative(2)

    totT = 0.0
    msg = JointTrajectory()
    # msg.header.frame_id = '/vicon/spot/spot'
    msg.header.frame_id = '/odom'
    msg.header.stamp = rospy.Time.now()

    while totT < t[-1]:

        msg.points.append(JointTrajectoryPoint())
        msg.points[-1].positions.append(cX(totT))
        msg.points[-1].positions.append(cY(totT))

        msg.points[-1].velocities.append(vX(totT))
        msg.points[-1].velocities.append(vY(totT))

        msg.points[-1].accelerations.append(aX(totT))
        msg.points[-1].accelerations.append(aY(totT))

        msg.points[-1].time_from_start = rospy.Duration(totT)
        totT += .1

    rospy.loginfo("********* SENDING CIRCLE TRAJECTORY *********")
    traj_pub.publish(msg)


def main():
    global odom, traj_pub

    rospy.init_node("traj_publisher")
    rate = rospy.Rate(1)

    odom_sub = rospy.Subscriber("/odometry/filtered", Odometry, odomcb)
    traj_pub = rospy.Publisher("/reference_trajectory", JointTrajectory, queue_size=1)


    rospy.sleep(2)
    while not rospy.is_shutdown():

        rate.sleep()

        if odom is None:
            continue

        sendLine()
        # sendCircle()
        sys.exit()
        

if __name__ == "__main__":
    main()