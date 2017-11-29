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
from ViconTrackerPoseHandler import ViconTrackerPoseHandler

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
def socialNavigation(navx, navy, xtarget, ytarget, theta, hmm, robot):
    global turtlePose

    rospy.init_node('social_navigation', anonymous=True)
    vicon = ViconTrackerPoseHandler(None, None, "", 51023, "ScottsHead")

    #setup environment
    if theta == 1:
        theta = 0
    elif theta == 2:
        theta = math.pi
    elif theta == 3:
        theta = math.pi/2.0
    else:
        theta = -math.pi/2.0

    #move robot or turtle
    if robot:
        rospy.loginfo("Start Robot")

        #initialize grid setup and path
        motion = AStar()
        human = (3,4)
        htheta = 0
        motion.init_grid(-3, -3, 3.5, 3.5, 0.5, (navx,navy), (xtarget,ytarget), theta, human)
        rospy.loginfo("The Destination is: " + str((motion.end.x,motion.end.y)))
        min_path = motion.search()
        rospy.loginfo("The Path is: " + str(min_path))

        inp = raw_input("Press Enter to Start Navigation")

        tstart = rospy.get_time()
        velPub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        vel_msg = Twist()

        #keep track of old human and robot pose
        oldhx = human[0]
        oldhy = human[1]
        oldhdir = htheta

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
            rospy.sleep(1.1)
            vel_msg.angular.z = 0
            vel_msg.linear.x =  math.sqrt(((move[0] - prevPos[0])**2) + ((move[1] - prevPos[1])**2))/3
            velPub.publish(vel_msg)

            #stop moving
            rospy.sleep(1.4)
            vel_msg.angular.z = 0
            vel_msg.linear.x = 0
            velPub.publish(vel_msg)

            #step changes
            prevPos = move
            motion.start = motion.cells[move]

            #get human pose after move
            track = vicon.getPose()
            rospy.loginfo(track)
            hx = track[0] #round(track[0] * 2) / 2.0
            hy = track[1]
            hdir = track[2]
            
            #predict if using hmms or check for movement
            if hmm:
                min_path = predict(min_path, (hx,hy), hdir)
            else:
                #check conditions for replan
                dist = math.sqrt(((hx - oldhx)**2) + ((hy - oldhy)**2))
                angle = abs(hdir - oldhdir)
                if dist >= 0.2 or angle >= 0.3:
                    #redo change to cost of old human position
                    motion.sum_cost[(hx,hy)] = motion.oldCost
                    #update changes to cost
                    motion.oldCost = motion.sum_cost[(hx,hy)]
                    motion.sum_cost[(hx,hy)] += 100.0
                    #replan
                    min_path = motion.search()

            #update changes human variables
            oldhx = hx
            oldhy = hy
            oldhdir = hdir

        #face target turtle
        rospy.sleep(1.4)
        vel_msg.angular.z = math.atan2(move[1] - prevPos[1], move[0] - prevPos[0]) - prevDir
        vel_msg.linear.x =  0
        velPub.publish(vel_msg)

        #stop
        rospy.sleep(1.4)
        vel_msg.angular.z = 0
        vel_msg.linear.x =  0
        velPub.publish(vel_msg)
    else:
        rospy.loginfo("Start Turtle Simulation")

        #initialize grid and path
        motion = AStar()
        human = (2,4)
        htheta = 0
        motion.init_grid(1, 1, 12, 12, 1, (navx,navy), (xtarget,ytarget), theta, human, htheta)
        rospy.loginfo("The Destination is: " + str((motion.end.x,motion.end.y)))
        min_path = motion.search()
        rospy.loginfo("The Path is: " + str(min_path))

        inp = raw_input("Press Enter to Start Navigation")
        tstart = rospy.get_time()

        setup(navx, navy, xtarget, ytarget, theta)
        rospy.Subscriber('/nav_turtle/pose', Pose, getPose)
        velPub = rospy.Publisher('/nav_turtle/cmd_vel', Twist, queue_size=10)
        vel_msg = Twist()

        oldhx = human[0]
        oldhy = human[1]
        oldhdir = htheta

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

            #get human pose after move
            track = vicon.getPose()
            rospy.loginfo(track)
            hx = track[0] #round(track[0] * 2) / 2.0
            hy = track[1]
            hdir = track[2]
            
            #predict if using hmms or check for movement
            if hmm:
                min_path = predict(min_path, (hx,hy), hdir)
            else:
                #check conditions for replan
                dist = math.sqrt(((hx - oldhx)**2) + ((hy - oldhy)**2))
                angle = abs(hdir - oldhdir)
                if dist >= 0.2 or angle >= 0.3:
                    #redo change to cost of old human position
                    motion.sum_cost[(hx,hy)] = motion.oldCost
                    #update changes to cost
                    motion.oldCost = motion.sum_cost[(hx,hy)]
                    motion.sum_cost[(hx,hy)] += 100.0
                    #replan
                    min_path = motion.search()

            #update changes human variables
            oldhx = hx
            oldhy = hy
            oldhdir = hdir

        #face target turtle
        rospy.sleep(1)
        vel_msg.angular.z = math.atan2(ytarget - turtlePose.y, xtarget - turtlePose.x) - turtlePose.theta
        vel_msg.linear.x =  0
        velPub.publish(vel_msg)

    tend = rospy.get_time()
    tnav = tend - tstart
    rospy.loginfo("Started at: " + str(tstart))
    rospy.loginfo("Reached Destination at: " + str(tend))
    rospy.loginfo("Travelling took " + str(tnav))

if __name__ == '__main__':
    try:
        robot = True
        if robot:
            navx = 3
            navy = -3
            xtarget = 3
            ytarget = 2
            theta = 1
            hmm = False
            socialNavigation(navx,navy,xtarget, ytarget, theta, hmm, robot)
        else:
            navx = 1
            navy = 1
            xtarget = 5
            ytarget = 9
            theta = 1
            hmm = False
            socialNavigation(navx,navy,xtarget, ytarget, theta, hmm, robot)
    except rospy.ROSInterruptException:
        pass
