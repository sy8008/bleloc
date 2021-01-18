import requests
import json
import urllib2
import urllib
import time
import cv2
import rospy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Quaternion
import numpy as np
import Queue

import tf

def judgeQueueDiff(x_queue,thresh):
	x_array=np.array(x_queue.queue)
	for i in range(1,x_array.shape[0]):
		dist=np.linalg.norm(x_array[i]-x_array[i-1])
		if(i==x_array.shape[0]-1):
			print(dist)
		if(dist>thresh):
			return False
	return True

def calcDist(map,pt):
	return np.linalg.norm(map-pt,axis=1)



def getPose_cb(data):
	global yaw_delta,yaw_rec,flag_pose_cb_entered
	flag_pose_cb_entered=True
	pose_quaternion=[data.orientation.x,data.orientation.y,data.orientation.z,data.orientation.w]
	curr_rpy=tf.transformations.euler_from_quaternion(pose_quaternion)
	curr_yaw=curr_rpy[2]
	yaw_rec=curr_yaw-yaw_delta





url = "http://192.168.31.100:2345/bleloc"

dist_thresh=13.35*0.5

topomap_xy=np.array([[150,267],[175,110],[238,110],[308,110],[340,110]])
curr_pos_discre=np.array([])


# global variable used in callback function
yaw_delta=1.83777 # unit in radius
yaw_rec=0 # rectified yaw
flag_pose_cb_entered=False

#loc = urllib2.urlopen(url)
point_color = (0, 0, 255) # BGR

bleloc_p=Pose()

posXY_queue=Queue.Queue(3)

#topic name config:
imu_pose_topic_name="/sensor/imu"
ble_pose_topic_name="/sensor/ble_pose"

map=cv2.imread("/home/songyang/BlelocMap.png")

rospy.init_node('bleloc_pub_pose', anonymous=True)
rospy.Subscriber(imu_pose_topic_name,Pose,getPose_cb)
bleloc_p_pub = rospy.Publisher(ble_pose_topic_name, Pose, queue_size=1)


iter=0


while not rospy.is_shutdown():
	iter+=1
	# print(iter)
	draw_map=map.copy()
	loc = requests.get(url).json()
	# print(type(loc))
	PosX,PosY=loc[u'posX'],loc[u'posY']



	if(posXY_queue.full()):
		posXY_queue.get()

	posXY_queue.put(np.array([PosX,PosY]))



	if (posXY_queue.full()):
		curr_pos=posXY_queue.queue[2]
		if(judgeQueueDiff(posXY_queue,dist_thresh)):
			curr_pos_discre = topomap_xy[np.argmin(calcDist(topomap_xy,curr_pos)),:]



	cv2.circle(draw_map, (int(PosX),469-int(PosY)),1, (255, 0, 0), 4)
	# cv2.circle(draw_map, (int(posx_rec),469-int(posy_rec)),1, point_color, 4)
	# cv2.circle(draw_map, (int(posx_rec), 469 - 110), 1, (0, 0, 255), 4)
	# cv2.circle(draw_map, (int(posx_clip),469-int(posy_clip)),1, (255, 0, 0), 4)
	if(curr_pos_discre.size!=0):
		pt1=(int(curr_pos_discre[0]), int(curr_pos_discre[1]))
		pt2=(int(pt1[0]-np.cos(yaw_rec)*25),int(pt1[1]+np.sin(yaw_rec)*25))
		cv2.circle(draw_map, (pt1[0],469-pt1[1]), 1, (0, 0, 255), 4)
		cv2.arrowedLine(draw_map, (pt1[0], 469 -pt1[1]), (pt2[0], 469 -pt2[1]), (255, 0, 0), \
						thickness=2, line_type=cv2.LINE_4, shift=0, tipLength=0.3)
		q=tf.transformations.quaternion_from_euler(0,0,yaw_rec)
		bleloc_p.position.x=pt1[0]
		bleloc_p.position.y=pt1[1]
		bleloc_p.position.z=0
		bleloc_p.orientation=Quaternion(*q)
		bleloc_p_pub.publish(bleloc_p)

	cv2.imshow("map",draw_map)
	cv2.waitKey(1)
	# print(loc)
	time.sleep(1)

cv2.destroyAllWindows()






