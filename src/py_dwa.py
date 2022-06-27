#!/usr/bin/env python
from dis import dis
import enum
import rospy
from nav_msgs.msg import Odometry
import math
import config
import numpy as np
from tf.transformations import euler_from_quaternion
import matplotlib.pyplot as plt
from geometry_msgs.msg import Twist, Pose
from sensor_msgs.msg import LaserScan
import sys

raw_data_twist = Twist()
raw_data_pose = Pose()
scan = LaserScan()
scanUpdated = False

def draw_line(x, y, angle):
    r = 1  # or whatever fits you
    plt.arrow(x, y, r*math.cos(angle), r*math.sin(angle))


def genPoint(current, vel, yawRate, dt):
    # rospy.loginfo(current)
    current[3] = vel  # new angle
    current[2] += yawRate * dt  # new angle
    current[0] += vel * math.cos(current[2])*dt
    current[1] += vel * math.sin(current[2])*dt
    return current


def dynamicWindow(vel, yaw_rate, dt, max_acc, max_alpha, vel_constrain, yawrate_contrain):
    # [min,max,min_y,max_y]
    param = [max(vel-max_acc*dt, vel_constrain[0]), min(vel+max_acc*dt, vel_constrain[1]),
             max(yaw_rate-max_alpha*dt, -yawrate_contrain), min(yaw_rate+max_alpha*dt, yawrate_contrain)]
    return param


def heading_cost(trajToCheck, goal):
    xDiff = goal[0] - trajToCheck[-1, 0]
    yDiff = goal[1] - trajToCheck[-1, 1]

    targetAngle = math.atan2(yDiff, xDiff)
    costAngle = targetAngle - trajToCheck[-1, 2]
    return abs(costAngle)

def dist_cost(trajToCheck,goal):
    dist =  np.hypot(trajToCheck[-1,0] - goal[0], trajToCheck[-1,1] - goal[1])
    return dist#*(1/initialDistance)


def velocity_cost(currentSpeed, maxSpeed, currentAngular, maxAngular):
    return  (maxAngular - abs(currentAngular)) #+(maxSpeed - abs(currentSpeed))


def obstacle_cost(trajToCheck, obstacles):
    obsX = obstacles[:, 0]
    obsY = obstacles[:, 1]
    xDiff = np.zeros([0, trajToCheck.shape[0]])
    yDiff = np.zeros([0, trajToCheck.shape[0]])
    for obstacle in obstacles:
        xDiff = np.vstack((xDiff, [trajToCheck[:, 0] - obstacle[0]]))
        yDiff = np.vstack((yDiff, [trajToCheck[:, 1] - obstacle[1]]))
    euclidDistance = np.hypot(xDiff, yDiff)
    # check collision
    # print("Min Range",np.min(scan.ranges))

    dist = np.min(euclidDistance)

    yaw = trajToCheck[:, 2]
    rot = np.array([[np.cos(yaw), -np.sin(yaw)], [np.sin(yaw), np.cos(yaw)]])
    rot = np.transpose(rot, [2, 0, 1])
    local_ob = obstacles[:, None] - trajToCheck[:, 0:2]
    local_ob = local_ob.reshape(-1, local_ob.shape[-1])
    local_ob = np.array([np.dot(local_ob, x) for x in rot])
    local_ob = local_ob.reshape(-1, local_ob.shape[-1])
    upper_check = local_ob[:, 0] <= config.LENGTH_ROBOT / 2
    right_check = local_ob[:, 1] <= config.WIDTH_ROBOT / 2
    bottom_check = local_ob[:, 0] >= -config.LENGTH_ROBOT / 2
    left_check = local_ob[:, 1] >= -config.WIDTH_ROBOT / 2
    if (np.logical_and(np.logical_and(upper_check, right_check),
                        np.logical_and(bottom_check, left_check))).any():
        return float("Inf"), dist
    # if dist <= config.MAX_DIG_SIZE:
    # if (euclidDistance <= config.MAX_DIG_SIZE).any():
    #     return float("Inf"), dist
    return 1.0/dist, dist


def laserToObs(currentLocation):
    obstacles = []
    for i, radius in enumerate(scan.ranges):
        if radius is not float("inf") and radius <= config.LASER_THRESH:
            x = radius/config.FACTOR * math.cos(i*scan.angle_increment + scan.angle_min) + currentLocation[0]
            y = radius/config.FACTOR * \
                math.sin(i*scan.angle_increment + scan.angle_min) + \
                currentLocation[1]
            obstacles.append([x, y])
        # rospy.loginfo(len(scan.ranges))
        # rospy.loginfo(3.14159/scan.angle_increment)
    # rospy.loginfo(obstacles)
    obstacles = np.array(obstacles)
    return obstacles


def dwa(current, window, goal):
    obs = laserToObs(current)
    minCost = float("inf")
    bestTraj = np.array([current])
    bestMove = [0.0, 0.0]
    bestDist = 0.0
    bestObstacle = 0
    bestHeading = 0
    bestVel = 0
    i = 0
    for vel in np.arange(window[0], window[1], config.RESO):
        for yawRate in np.arange(window[2], window[3], config.RESO_angular):
            currentStatus = np.array(current)
            traj = np.array(currentStatus)  # array of dt goal points is traj
            for _ in np.arange(0, config.TIME_CALC, config.DT):
                currentStatus = genPoint(
                    currentStatus, vel, yawRate, config.DT)
                traj = np.vstack((traj, currentStatus))  # new row add

            # plt.plot(traj[:,0],traj[:,1],'g')

            obstacleDistance, dist = obstacle_cost(traj, obs)
            headingCost = heading_cost(traj, goal)
            distCost = dist_cost(traj, goal)
            velocityCost = velocity_cost(
                vel, config.MAX_VEL, yawRate, config.MAX_YAWRATE)
            finalCost = obstacleDistance * config.OBSTACLEGAIN + velocityCost * \
                config.VELOCITYGAIN + headingCost * config.HEADINGGAIN + distCost * config.DISTANCEGAIN
            # print(finalCost)

            if minCost >= finalCost:
                minCost = finalCost
                bestMove = [vel, yawRate]
                bestTraj = traj
                bestDist = distCost * config.DISTANCEGAIN
                bestObstacle = obstacleDistance * config.OBSTACLEGAIN
                bestHeading = headingCost * config.HEADINGGAIN
                bestVel = velocityCost * config.VELOCITYGAIN
                if (abs(bestMove[0]) < config.STUCK) and (abs(currentStatus[3]) < config.STUCK):
                    bestMove[0] = config.MAX_ACC/10

            # scan_portion = np.array(scan.ranges[:len(scan.ranges)/2-60])
            # scan_portion_centre = np.array(scan.ranges[len(scan.ranges)/2-60:len(scan.ranges)/2+60])
            # if (abs(bestMove[0]) < config.STUCK) and (abs(currentStatus[3]) < config.STUCK) and finalCost == float("inf"):
            #     bestMove[0]  = 0
            # if not (scan_portion_centre <= config.MAX_DIG_SIZE).any():
            #     print("Centre")
            #     # bestMove[1]  = 0
            #     # bestMove[0]  = config.MAX_ACC
            # elif (scan_portion <= config.MAX_DIG_SIZE).any():
            #     # bestMove[1]  = config.MAX_ALPHA/5
            #     print("Left")
            # else:
            #     print("Right")
            #     # bestMove[1]  = -config.MAX_ALPHA/5


            # elif obstacleDistance == float("inf"):
            #     print("COLLISION!!")
            # if finalCost == float("inf"):
            #     bestMove = [0.0, 0.1]
            # rospy.loginfo(yawRate)
            # rospy.loginfo(vel)
            # traj = predict_trajectory(current, 1.0, 1.8)
            # rospy.loginfo(traj)
            # plt.plot(traj[:, 0], traj[:, 1])
            # plt.savefig(str(i)+".png")
            # i = i+1
            # plt.show()
            # rospy.loginfo(traj[:, 0])
            # draw_line(data[0][0], data[0][1], data[0][2])
    # plt.plot(bestTraj[:, 0], bestTraj[:, 1],'r')
    # i = i+1
    # plt.savefig(str(i)+".png")



    rospy.loginfo("Obstacle distance"+ str(dist))
    rospy.loginfo("Obstacle Cost"+ str( bestObstacle))
    rospy.loginfo("Heading Cost"+ str( bestHeading))
    rospy.loginfo("Dist Cost"+ str( bestDist))
    rospy.loginfo("Velocity Cost"+ str( bestVel))
    rospy.loginfo(str(bestMove)+"Score" + str( minCost))
    # plt.plot(bestTraj[:, 0], bestTraj[:, 1])
    # plt.show()
    return bestMove, bestTraj





def callback(data):
    global raw_data_twist, raw_data_pose
    raw_data_twist = data.twist.twist
    raw_data_pose = data.pose.pose


def scan_callback(data):
    global scanUpdated
    global scan
    scan = data
    scanUpdated = True


def genTrajectory():

    rospy.init_node('py_dwa', anonymous=True)

    rospy.Subscriber("/odometry/filtered", Odometry, callback)
    rospy.Subscriber('/scan_filtered', LaserScan, scan_callback)
    rate = rospy.Rate(10000)
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)

    # rospy.loginfo(genPoint([0, 0, 0], 1, 1, 0.5))
    twist = Twist()
    i = 0
    while not rospy.is_shutdown():
        # rospy.loginfo(raw_data_pose)
        # rospy.loginfo(raw_data_twist.angular.z)
        start = rospy.get_time()

        dynamicWindowCurrent = dynamicWindow(
            raw_data_twist.linear.x, raw_data_twist.angular.z, config.DT, config.MAX_ACC, config.MAX_ALPHA, [config.MIN_VEL, config.MAX_VEL], config.MAX_YAWRATE)
        # rospy.loginfo(raw_data_pose)
        rospy.loginfo("###################")
        rospy.loginfo("Current Location"+str(raw_data_pose.position.x)+str(raw_data_pose.position.y))
        (roll, pitch, yaw) = euler_from_quaternion(
            [raw_data_pose.orientation.x, raw_data_pose.orientation.y, raw_data_pose.orientation.z, raw_data_pose.orientation.w])
        # plt.close()
        if scanUpdated:
            distToGoal = math.hypot(raw_data_pose.position.x - config.GOAL[0],raw_data_pose.position.y - config.GOAL[1])
            if distToGoal <= config.THRESHOLD:
                print("Goal!")
                twist.linear.x = 0
                twist.angular.z = 0
                pub.publish(twist)
                raise rospy.exceptions.ROSInterruptException

            else:
                move, traj = dwa([raw_data_pose.position.x,
                                  raw_data_pose.position.y, yaw, 0], dynamicWindowCurrent, config.GOAL)
                plt.plot(traj[:, 0], traj[:, 1], 'r')
                i = i+1

                twist.linear.x = move[0]
                twist.angular.z = move[1]

            pub.publish(twist)

            # trajectory = np.vstack((trajectory, x))
        # plt.show()
        rospy.loginfo((rospy.get_time())-start)
        start = rospy.get_time()
        # rate.sleep()
    rospy.spin()


if __name__ == '__main__':
    try:
        genTrajectory()
    except rospy.exceptions.ROSInterruptException:
        print("BYEEE")
        plt.savefig("1_RUN.png")
        sys.exit()
