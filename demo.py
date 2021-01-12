import requests
import json
import urllib2
import urllib
import time
import cv2
import rospy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose

import numpy as np


def getVelocity_cb(data):
	global linear_vel,angle_vel
	linear_vel=np.array([[data.linear.x,data.linear.y,data.linear.z]],dtype=np.float32)
	angle_vel=np.array([[data.angular.x,data.angular.y,data.angular.z]],dtype=np.float32)
	# print("linear_vel:{}".format(linear_vel))

	
global linear_vel,angle_vel
url = "http://192.168.31.100:2345/bleloc"
#loc = urllib2.urlopen(url)
point_color = (0, 0, 255) # BGR
linear_vel=np.zeros((1,3),dtype=np.float32)
angle_vel=np.zeros((1,3),dtype=np.float32)
bleloc_cmd=Twist()
bleloc_p=Pose()
kalman=cv2.KalmanFilter(4,2)
kalman.measurementMatrix=np.array([[1,0,0,0],[0,1,0,0]],np.float32)
kalman.transitionMatrix=np.array([[1,0,1,0],[0,1,0,1],[0,0,1,0],[0,0,0,1]],np.float32)
kalman.processNoiseCov=np.array([[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]],np.float32)*3
kalman.measurementNoiseCov=np.array([[1,0],[0,1]],np.float32)*0.01




map=cv2.imread("/home/songyang/BlelocMap.png")

rospy.init_node('bleloc', anonymous=True)
rospy.Subscriber("/cmd_vel", Twist, getVelocity_cb)
bleloc_p_pub = rospy.Publisher('/pose', Pose, queue_size=1)


while not rospy.is_shutdown():
	draw_map=map.copy()
	loc = requests.get(url).json()
	# print(type(loc))
	PosX,PosY=loc[u'posX'],loc[u'posY']

	kalman.correct(np.array([[PosX],[PosY]],np.float32))
	prediction=kalman.predict()

	bleloc_p.position.x = PosX
	bleloc_p.position.y = PosY
	bleloc_p.position.z = 0.0
	# Make sure the quaternion is valid and normalized
	bleloc_p.orientation.x = 0.0
	bleloc_p.orientation.y = 0.0
	bleloc_p.orientation.z = 0.0
	bleloc_p.orientation.w = 0.0
	bleloc_p_pub.publish(bleloc_p)

	cv2.circle(draw_map, (int(PosX),469-int(PosY)),1, point_color, 4)
	cv2.circle(draw_map, (int(prediction[0,0]),469-int(prediction[1,0])),1, (255, 255, 0), 4)

	cv2.imshow("map",draw_map)
	cv2.waitKey(1)
	# print(loc)
	time.sleep(1)

cv2.destroyAllWindows()






