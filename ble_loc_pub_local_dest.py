import time
import cv2
import rospy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Quaternion
import numpy as np
import Queue

import tf




def calcDist(map,pt):
	return np.linalg.norm(map-pt,axis=1)

def getPose_cb(data):
	global curr_yaw,flag_pose_cb_entered,curr_position_xy
	flag_pose_cb_entered=True
	pose_quaternion=[data.orientation.x,data.orientation.y,data.orientation.z,data.orientation.w]
	curr_rpy=tf.transformations.euler_from_quaternion(pose_quaternion)
	curr_yaw=curr_rpy[2]
	curr_position_xy=np.array([int(data.position.x),int(data.position.y)])


map=cv2.imread("/home/songyang/BlelocMap.png")

curr_yaw=0
flag_pose_cb_entered=False
curr_position_xy=[]
dest_pos_xy_bf=[] # desitination in bodyframe corrdinate
ble_pose_topic_name="/sensor/ble_pose"
ble_dest_pose_topic_name="sensor/ble_dest_pose"
rospy.init_node('bleloc_pub_local_dest', anonymous=True)
rospy.Subscriber(ble_pose_topic_name,Pose,getPose_cb)
bleloc_dest_p_pub = rospy.Publisher(ble_dest_pose_topic_name, Pose, queue_size=1)
bleloc_dest_pos=Pose()
topomap_xy=np.array([[150,267],[175,110],[238,110],[308,110],[340,110]])


while not rospy.is_shutdown():
	draw_map=map.copy()
	white_draw_map=np.ones(draw_map.shape,np.uint8)*255
	if(len(curr_position_xy)!=0):
		cur_pos_idx=np.argmin(calcDist(topomap_xy, np.array(curr_position_xy)))
		if(cur_pos_idx!=0):
			dest_pos_xy=topomap_xy[1,:]
		else:
			dest_pos_xy=topomap_xy[0,:]

		dist=np.linalg.norm(dest_pos_xy-curr_position_xy)
		dest_pos_xy_bf=[int(dist*np.cos(curr_yaw)),-int(dist*np.sin(curr_yaw))]
		bleloc_dest_pos.position.x=dest_pos_xy_bf[0]
		bleloc_dest_pos.position.y=dest_pos_xy_bf[1]
		bleloc_dest_pos.position.z=0
		bleloc_dest_pos.orientation=Quaternion(0,0,0,0)
		pt2_x_direct=[int(curr_position_xy[0]-np.cos(curr_yaw)*50),int(curr_position_xy[1]+np.sin(curr_yaw)*50)]
		pt2_y_derect=[int(curr_position_xy[0]-np.sin(curr_yaw)*50),int(curr_position_xy[1]-np.cos(curr_yaw)*50)]


		# cv2.arrowedLine(draw_map, (curr_position_xy[0], 469 - curr_position_xy[1]), (pt2_x_direct[0], 469 -pt2_x_direct[1]), (255, 0, 0), \
		# 				thickness=2, line_type=cv2.LINE_4, shift=0, tipLength=0.3)
		# cv2.arrowedLine(draw_map, (curr_position_xy[0], 469 - curr_position_xy[1]), (pt2_y_derect[0], 469 -pt2_y_derect[1]), (0, 0, 255), \
		# 				thickness=2, line_type=cv2.LINE_4, shift=0, tipLength=0.3)

		xy_origin=[20,450] # The axis orgin to draw in the image
		x_pt2=[300,450]
		y_pt2=[20,170]
		x_text_origin=[350,450]
		y_text_origin=[20,150]
		# draw x axis
		cv2.arrowedLine(white_draw_map, tuple(xy_origin), tuple(x_pt2), (255, 0, 0), \
						thickness=2, line_type=cv2.LINE_4, shift=0, tipLength=0.05)
		cv2.putText(white_draw_map,"x",tuple(x_text_origin),cv2.FONT_HERSHEY_COMPLEX,0.5,(255, 0, 0),2)

		# draw y axis
		cv2.arrowedLine(white_draw_map, tuple(xy_origin), tuple(y_pt2), (0, 0, 255), \
						thickness=2, line_type=cv2.LINE_4, shift=0, tipLength=0.05)
		cv2.putText(white_draw_map,"y",tuple(y_text_origin),cv2.FONT_HERSHEY_COMPLEX,0.5,(255, 0, 0),2)


		# cv2.arrowedLine(white_draw_map, (curr_position_xy[0], 469 - curr_position_xy[1]), (pt2_y_derect[0], 469 -pt2_y_derect[1]), (0, 0, 255), \
		# 				thickness=2, line_type=cv2.LINE_4, shift=0, tipLength=0.3)

		cv2.circle(draw_map, (curr_position_xy[0], 469 - curr_position_xy[1]), 1, (255, 0, 0), 4) # draw current position in global map
		cv2.circle(draw_map, (curr_position_xy[0], 469 - curr_position_xy[1]), 1, (255, 0, 0), 4) # draw local destination in global map
		draw_dest_pos_xy_bf=[dest_pos_xy_bf[0]+xy_origin[0],-dest_pos_xy_bf[1]+xy_origin[1]]
		cv2.circle(white_draw_map, tuple(draw_dest_pos_xy_bf), 3, (0, 0, 0), 4) # draw local destination in body frame
		merged_map_to_show=np.zeros((draw_map.shape[0],draw_map.shape[1]*2,3),np.uint8)
		merged_map_to_show[:,0:draw_map.shape[1],:]=draw_map
		merged_map_to_show[:,draw_map.shape[1]:,:]=white_draw_map

		flag_pose_cb_entered=False
		# cv2.imshow("local destination map", white_draw_map)
		# cv2.imshow("global destination map",draw_map)
		cv2.imshow("local destination map",merged_map_to_show)
		cv2.waitKey(1)

cv2.destroyAllWindows()