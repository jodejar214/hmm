#!/usr/bin/env python
import rospy
import math
from aStar import Cell
from aStar import AStar
from random import randint
from turtlesim.srv import Spawn
from turtlesim.srv import Kill
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist

velPub = None
vel_msg = None
turtlePose = None

#update navigating turtle's pose
def getPose(data):
    global turtlePose
    turtlePose = data

#setups turtlesim grid
def setup(navx, navy, xtarget, ytarget, theta):
    #kill default turtle
    turtle_kill = rospy.ServiceProxy('kill', Kill)
    try:
        turtle_kill('turtle1')
    except rospy.ServiceException as ex:
        rospy.logerr('Could not kill turtle! Exception: {}'.format(ex))
        rospy.signal_shutdown("Can't kill...")

    #spawn navigating and target turtle
    rospy.wait_for_service('spawn')
    turtle_spawn = rospy.ServiceProxy('spawn', Spawn)
    try:
        t1 = turtle_spawn(navx, navy, 0, 'nav_turtle').name
        t2 = turtle_spawn(xtarget, ytarget, theta, 'target_turtle').name
    except rospy.ServiceException as ex:
        rospy.logerr('Could not spawn turtle! Exception: {}'.format(ex))
        rospy.signal_shutdown("Can't spawn...")

    rospy.loginfo("Starting Position:"+str((navx,navy)))
    rospy.loginfo("Target Turtle:"+str((xtarget,ytarget))+" , Theta:"+str(theta))

#move navigating turtle to target in grid with costs
def socialNavigation(navx,navy,xtarget, ytarget, theta, hmm):
    global turtlePose, velPub, vel_msg

    rospy.init_node('social_navigation', anonymous=True)

    #setup environment
    if theta == 1:
        theta = 0
    elif theta == 2:
        theta = math.pi
    elif theta == 3:
        theta = math.pi/2.0
    else:
        theta = -math.pi/2.0

    setup(navx, navy, xtarget, ytarget, theta)

    #initial grid setup and path
    motion = AStar()
    motion.init_grid(12, 12, (navx,navy), (xtarget,ytarget), theta, (3,4))
    rospy.loginfo("The Destination is: " + str((motion.end.x,motion.end.y)))
    min_path = motion.search()

    if robot:
        velPub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        vel_msg = Twist()

        prevDir = 0
        prevPos = (navx,navy)

        #move towards the endpoint
        while len(min_path) > 0:
            move = min_path.pop(0)

            #face towards target
            rospy.sleep(1)
            vel_msg.angular.z = math.atan2(move[1] - prevPos[1], move[0] - prevPos[0]) - prevDir
            vel_msg.linear.x =  0
            velPub.publish(vel_msg)

            prevDir += vel_msg.angular.z

            #move towards target
            rospy.sleep(1)
            vel_msg.angular.z = 0
            vel_msg.linear.x =  math.sqrt(((move[0] - prevPos[0])**2) + ((move[1] - prevPos[1])**2))
            velPub.publish(vel_msg)

            prevPos = move

        #face target turtle
        rospy.sleep(1)
        vel_msg.angular.z = math.atan2(ytarget - turtlePose.y, xtarget - turtlePose.x) - turtlePose.theta
        vel_msg.linear.x =  0
        velPub.publish(vel_msg)
    else:
        rospy.Subscriber('/nav_turtle/pose', Pose, getPose)
        velPub = rospy.Publisher('/nav_turtle/cmd_vel', Twist, queue_size=10)
        vel_msg = Twist()

        #move towards the endpoint
        while len(min_path) > 0:
            move = min_path.pop(0)

            #face towards target
            rospy.sleep(1)
            vel_msg.angular.z = math.atan2(move[1] - turtlePose.y, move[0] - turtlePose.x) - turtlePose.theta
            vel_msg.linear.x =  0
            velPub.publish(vel_msg)

            #move towards target
            rospy.sleep(1)
            vel_msg.angular.z = 0
            vel_msg.linear.x =  math.sqrt(((move[0] - turtlePose.x)**2) + ((move[1] - turtlePose.y)**2))
            velPub.publish(vel_msg)

        #face target turtle
        rospy.sleep(1)
        vel_msg.angular.z = math.atan2(ytarget - turtlePose.y, xtarget - turtlePose.x) - turtlePose.theta
        vel_msg.linear.x =  0
        velPub.publish(vel_msg)

    # #use hmm to predict future position or use current position for path costs
    # if hmm:
    #     min_path = motion.predict()
    # else:
    #     motion.humanCost(motion.human, math.pi)
    #     min_path = motion.search()

    # rospy.loginfo("The Path is: " + str(min_path))
    # tstart = rospy.get_time()

    # #move towards the endpoint
    # while len(min_path) > 0:
    #     move = min_path.pop(0)

    #     #face towards target
    #     rospy.sleep(1)
    #     vel_msg.angular.z = math.atan2(move[1] - turtlePose.y, move[0] - turtlePose.x) - turtlePose.theta
    #     vel_msg.linear.x =  0
    #     velPub.publish(vel_msg)

    #     #move towards target
    #     rospy.sleep(1)
    #     vel_msg.angular.z = 0
    #     vel_msg.linear.x =  math.sqrt(((move[0] - turtlePose.x)**2) + ((move[1] - turtlePose.y)**2))
    #     velPub.publish(vel_msg)

    #     #choose to change path for avoindance after each step
    #     if hmm:
    #         min_path = motion.predict()
    #     else:
    #         motion.humanCost(motion.human, 0)
    #         min_path = motion.search()

    # #face target turtle
    # rospy.sleep(1)
    # vel_msg.angular.z = math.atan2(ytarget - turtlePose.y, xtarget - turtlePose.x) - turtlePose.theta
    # vel_msg.linear.x =  0
    # velPub.publish(vel_msg)

    # tend = rospy.get_time()
    # tnav = tend - tstart
    # rospy.loginfo("Started at: " + str(tstart))
    # rospy.loginfo("Reached Destination at: " + str(tend))
    # rospy.loginfo("Travelling took " + str(tnav))

if __name__ == '__main__':
    try:
        socialNavigation(1,1,7,7,1, False, True)
    except rospy.ROSInterruptException:
        pass
