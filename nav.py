#!/usr/bin/env python
import rospy
import math
import numpy as np
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
def setup(navx, navy, xtarget, ytarget, theta, xobs, yobs):
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
        t3 = turtle_spawn(xobs, yobs, 0, 'obstacle').name
    except rospy.ServiceException as ex:
        rospy.logerr('Could not spawn turtle! Exception: {}'.format(ex))
        rospy.signal_shutdown("Can't spawn...")

    rospy.loginfo("Starting Position:"+str((navx,navy)))
    rospy.loginfo("Target Turtle:"+str((xtarget,ytarget))+" , Theta:"+str(theta))
    rospy.loginfo("Obstacle Turtle:"+str((xobs,yobs)))

#move navigating turtle to target in grid with costs
def socialNavigation(navx, navy, xtarget, ytarget, theta, hmm, robot, subj):
    global turtlePose

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

    pathCalcCount = 0

    #move robot or turtle
    if robot:
        rospy.loginfo("Start Robot")

        #initialize grid setup and path
        vicon = ViconTrackerPoseHandler(None, None, "", 51040, "helmet")
        motion = AStar()
        human = (0,0)
        htheta = 0
        motion.init_grid(-3., -3., 3.5, 3.5, 0.5, (navx,navy), (xtarget,ytarget), theta, human)
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

        oldneighbors = motion.get_neighbors(motion.cells[(oldhx,oldhy)])
        for n in oldneighbors:
            motion.sum_cost[(n.x,n.y)] += 100.0
            motion.changed.append((n.x,n.y))

        #move towards the endpoint
        rospy.sleep(1)
        while len(min_path) > 0:
            move = min_path.pop(0)

            #face towards target
            vel_msg.angular.z = math.atan2(move[1] - prevPos[1], move[0] - prevPos[0]) - prevDir
            vel_msg.linear.x =  0

            prevDir += vel_msg.angular.z

            angle = round(math.degrees(vel_msg.angular.z)/45)*45
            neg = 1
            if angle < 0: 
                neg = -1
            if angle == 45 or angle == -45:
                vel_msg.angular.z -= 0.02*neg
            elif angle == 90 or angle == -90:
                vel_msg.angular.z -= 0.04*neg
            elif angle == 135 or angle == -135:
                vel_msg.angular.z -= 0.1*neg
            else:
                vel_msg.angular.z -= 0.13*neg

            velPub.publish(vel_msg)
            rospy.sleep(1.11)

            #move towards target
            vel_msg.angular.z = 0
            vel_msg.linear.x = math.sqrt(((move[0] - prevPos[0])**2) + ((move[1] - prevPos[1])**2))
            velPub.publish(vel_msg)
            rospy.sleep(1.3)

            #stop moving
            vel_msg.angular.z = 0
            vel_msg.linear.x = 0
            velPub.publish(vel_msg)
            rospy.sleep(1)

            #step changes
            prevPos = move

            #get human pose after move
            track = vicon.getPose()
            rospy.loginfo("Vicon Pose: " + str(track))
            hx = round(track[0] * 2) / 2 #round(track[0] * 2) / 2.0
            hy = round(track[1] * 2) / 2
            hdir = math.radians(round(math.degrees(track[2])/45)*45)
            
            motion.sum_cost[(oldhx,oldhy)] -= 100.0
            motion.sum_cost[(hx,hy)] += 100.0

            #predict if using hmms or check for movement
            if len(min_path) > 0:
                if hmm:
                    #redo changes
                    for c in motion.changed:
                        motion.sum_cost[c] -= 100.0
                    motion.changed = []

                    if len(min_path) > 1:
                        #replan if high prob of collision
                        change = motion.pathProbs(min_path, (hx,hy), hdir)
                        if change:
                            motion.start = motion.cells[move]
                            min_path = motion.search()
                            rospy.loginfo("The New Path is: " + str(min_path))
                            pathCalcCount += 1
                else:
                    #check conditions for replan
                    dist = math.sqrt(((hx - oldhx)**2) + ((hy - oldhy)**2))
                    angle = abs(hdir - oldhdir)

                    #undo changes to cost
                    for n in oldneighbors:
                        motion.sum_cost[(n.x,n.y)] -= 100.0

                    if dist >= 0.2 or angle >= 0.3:
                        #update changes to cost
                        neighbors = motion.get_neighbors(motion.cells[(hx,hy)])
                        for n in neighbors:
                            motion.sum_cost[(n.x,n.y)] += 100.0
                        oldneighbors = neighbors[:]

                        #replan
                        motion.start = motion.cells[move]
                        min_path = motion.search()
                        rospy.loginfo("The New Path is: " + str(min_path))
                        pathCalcCount += 1

            #update changes human variables
            oldhx = hx
            oldhy = hy
            oldhdir = hdir

        #face target turtle
        tend = rospy.get_time()
        vel_msg.angular.z = math.atan2(move[1] - prevPos[1], move[0] - prevPos[0]) - prevDir
        vel_msg.linear.x =  0

        angle = round(math.degrees(vel_msg.angular.z)/45)*45
        if angle == 45 or angle == -45:
            vel_msg.angular.z -= 0.02
        elif angle == 90 or -90:
            vel_msg.angular.z -= 0.04
        elif angle == 135 or -135:
            vel_msg.angular.z -= 0.1
        else:
            vel_msg.angular.z -= 0.13

        velPub.publish(vel_msg)
        rospy.sleep(1.11)

        #stop
        vel_msg.angular.z = 0
        vel_msg.linear.x =  0
        velPub.publish(vel_msg)
        rospy.sleep(1)

    else:
        rospy.loginfo("Start Turtle Simulation")

        #initialize grid and path
        motion = AStar()
        human = (2,4)
        htheta = math.radians(0)
        motion.init_grid(1, 1, 12, 12, 1, (navx,navy), (xtarget,ytarget), theta, human)
        rospy.loginfo("The Destination is: " + str((motion.end.x,motion.end.y)))
        min_path = motion.search()
        rospy.loginfo("The Path is: " + str(min_path))

        inp = raw_input("Press Enter to Start Navigation")
        tstart = rospy.get_time()

        #setup subjects, goal, and nav
        setup(navx, navy, xtarget, ytarget, theta, human[0], human[1])
        rospy.Subscriber('/nav_turtle/pose', Pose, getPose)
        velPub = rospy.Publisher('/nav_turtle/cmd_vel', Twist, queue_size=10)
        vel_msg = Twist()
        # obs_path = [(2,4,0),(3,4,0),(4,4,0),(5,4,math.radians(90)),(5,5,0),(4,6,0),(4,7,0),(4,8,0)]
        # i = 0

        #keep track of old human pose
        oldhx = human[0]
        oldhy = human[1]
        oldhdir = htheta

        motion.changed.append(human)
        oldneighbors = motion.get_neighbors(motion.cells[(oldhx,oldhy)])
        for n in oldneighbors:
            motion.sum_cost[(n.x,n.y)] += 100.0
            motion.changed.append((n.x,n.y))

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
            # if i < len(obs_path):
            track = (2,4,math.radians(0))
            # i+=1
            rospy.loginfo(track)
            hx = track[0]
            hy = track[1]
            hdir = track[2]

            motion.sum_cost[(oldhx,oldhy)] -= 100.0
            motion.sum_cost[(hx,hy)] += 100.0
            
            #predict if using hmms or check for movement
            if len(min_path) > 0:
                if hmm:
                    #redo changes
                    for c in motion.changed:
                        motion.sum_cost[c] -= 100.0
                    motion.changed = []

                    if len(min_path) > 1:
                        #replan if high prob of collision
                        change = motion.pathProbs(min_path, (hx,hy), hdir)
                        if change:
                            motion.start = motion.cells[move]
                            min_path = motion.search()
                            rospy.loginfo("The New Path is: " + str(min_path))

                else:
                    #check conditions for replan
                    dist = math.sqrt(((hx - oldhx)**2) + ((hy - oldhy)**2))
                    angle = abs(hdir - oldhdir)

                    #undo changes to cost
                    for n in oldneighbors:
                        motion.sum_cost[(n.x,n.y)] -= 100.0

                    if dist >= 0.2 or angle >= 0.3:
                        #update changes to cost
                        neighbors = motion.get_neighbors(motion.cells[(hx,hy)])
                        for n in neighbors:
                            motion.sum_cost[(n.x,n.y)] += 100.0
                        oldneighbors = neighbors[:]

                        #replan
                        motion.start = motion.cells[move]
                        min_path = motion.search()
                        rospy.loginfo("The New Path is: " + str(min_path))

            #update changes human variables
            oldhx = hx
            oldhy = hy
            oldhdir = hdir

        #face target turtle
        tend = rospy.get_time()
        rospy.sleep(1)
        vel_msg.angular.z = math.atan2(ytarget - turtlePose.y, xtarget - turtlePose.x) - turtlePose.theta
        vel_msg.linear.x =  0
        velPub.publish(vel_msg)

    tnav = tend - tstart
    rospy.loginfo("Travelling took " + str(tnav))
    rospy.loginfo("The Total Number of Path Calculations: " + str(pathCalcCount))

    #write observations to file
    with open("results.txt","a") as f:
        f.write("Subject "+str(subj)+"\n")
        f.write("Condition = HMM "+str(hmm)+"\n")
        f.write("Path Intersect?\n")
        f.write("Collision?\n")
        f.write("The Total Number of Path Calculations: " + str(pathCalcCount) + "\n")
        f.write("Travelling took " + str(tnav) + "\n\n\n")

if __name__ == '__main__':
    try:
        robot = True
        if robot:
            navx = 2.5
            navy = 0
            xtarget = -0.5
            ytarget = -2.5
            theta = 1
            hmm = False
            subj = 10
            socialNavigation(navx,navy,xtarget, ytarget, theta, hmm, robot, subj)
        else:
            navx = 1
            navy = 1
            xtarget = 5
            ytarget = 9
            theta = 1
            hmm = True
            subj = 1
            socialNavigation(navx,navy,xtarget, ytarget, theta, hmm, robot, subj)
    except rospy.ROSInterruptException:
        pass
