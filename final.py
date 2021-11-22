#!/usr/bin/env python3
import cv2
import rospy
import actionlib
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

def check1(a,b,c,d):
	client= actionlib.SimpleActionClient('move_base',MoveBaseAction) 
	client.wait_for_server()
	goal2=MoveBaseGoal()
	goal2.target_pose.header.frame_id="map"
	goal2.target_pose.header.stamp=rospy.Time.now()
	goal2.target_pose.pose.position.x = a
	goal2.target_pose.pose.position.y = b
	goal2.target_pose.pose.orientation.z = c
	goal2.target_pose.pose.orientation.w = d
	client.send_goal(goal2)
	wait = client.wait_for_result()
	if not wait:
		rospy.logerr("Action server not available!")
		rospy.signal_shutdown("Action server not available!")
	else:
		print(client.get_result())


def right(): #Path to take after arrow direction detected if Right
    check1( -0.213,0.489,-0.999, 0.014)
    check1(-0.315,0.575,0.856,0.5)
    check1(-0.345,0.646,0.769,0.639)
    check1(-0.39,0.397,-0.999,0.024)
    check1(-0.843,0.959,-0.709,0.705)
    check1(-0.795,0.634,-0.688,0.725)
    check1(-0.792,0.628,-0.701,0.713)

def left(): #Path to take after arrow dirn detectn if LEft
    check1(-0.206,0.476,0.999,0.021)
    check1(-0.369,0.14,-0.698,0.716)
    check1(-8.59,0.0281,0.999,0.016)
    

def image_callback(msg: Image):
	cv_bridge = CvBridge()
	cv_img = cv_bridge.imgmsg_to_cv2(msg)
	cv2.imshow("cv",cv_img)
	
	hsv=cv2.cvtColor(cv_img, cv2.COLOR_BGR2HSV)
	img = cv2.GaussianBlur(cv_img, (11,11), 0)
	blue_lower = np.array([90, 60, 0], np.uint8)
	blue_upper = np.array([121, 255, 255], np.uint8)
	blue_mask = cv2.inRange(hsv, blue_lower, blue_upper)
	corners = cv2.goodFeaturesToTrack(blue_mask,7,0.01,10)
	corners = np.int0(corners)
	ct='a'
	for i in corners:
		x,y = i.ravel()
		print(x,y)
		cv2.circle(img,(x,y),3,(255,255,0),-1)
		cv2.putText(img, ct, (x,y), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0,0,255), 2, cv2.LINE_AA )
		ct=ct+'a'	
	xmax, ymax = (np.max(corners, axis = 0)).ravel()
	xmin, ymin = (np.min(corners, axis = 0)).ravel()
	if( abs(xmax-xmin) > abs(ymax-ymin)):
	    if(np.count_nonzero(corners[:,0,0] == xmax) == 2):
	        print('LEFT')
	        left()
	    else:
	        print('RIGHT')
	        right()
	else:
	    if(np.count_nonzero(corners[:,0,1] == ymax) == 2):
	        print('UP')
	    else:
	        print('DOWN')   

	cv2.imshow('image',img)
	k=cv2.waitKey(0) & 0xff
	if k==27:
	    cv2.destroyAllWindows()
	
def movebase_client(p):

    client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
    client.wait_for_server()

    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = p[0]
    goal.target_pose.pose.position.y = p[1]
    goal.target_pose.pose.orientation.z = p[2]
    goal.target_pose.pose.orientation.w = p[3]

    client.send_goal(goal)
    wait = client.wait_for_result()
    if not wait:
        rospy.logerr("Action server not available!")
        rospy.signal_shutdown("Action server not available!")
    else:
        return client.get_result()

def go_right(): #Initial Angling for Path to Arrow
	pr={0:[-0.41321,0.94478,-0.99998,0.00548] , 1:[-0.81984,0.76385,-0.71455,0.69957]}
	for i in [0,1]:
		try:
			rospy.init_node('movebase_client_py')
			result = movebase_client(ps[i])
			if result:
				rospy.loginfo("Goal execution done!")
		except rospy.ROSInterruptException:
			rospy.loginfo("Navigation test finished.")
			
def go_left(): #Initial angling for path to Arrow
	pr={0:[-0.38815,-0.00520,0.99999,0.00258] , 1:[-0.70581,-0.00520,0.72400,0.68979]}
	for i in [0,1]:
		try:
			rospy.init_node('movebase_client_py')
			result = movebase_client(ps[i])
			if result:
				rospy.loginfo("Goal execution done!")
		except rospy.ROSInterruptException:
			rospy.loginfo("Navigation test finished.")

	

if __name__ == '__main__': #PATH TO THE ARROW - 5 CHECKPOINTS/WAYPOINTS
	ps={0:[0.536,0.0042,0.70299,0.7111] , 1:[0.48103,0.9472,0.99998,0.005888] , 2:[0.04315,0.9596,-0.73154,0.68179] , 3:[0.09083,0.4574,0.99998,0.00563] , 4:[-0.1,0.45122,0.99998,0.00585]}
	for i in [0,1,2,3,4]:
		try:
			rospy.init_node('movebase_client_py')
			result = movebase_client(ps[i])
			if result:
				rospy.loginfo("Goal execution done!")
		except rospy.ROSInterruptException:
			rospy.loginfo("Navigation test finished.")
	rospy.Subscriber("camera/image_raw",Image,image_callback)
	rospy.spin()
