#Gerritt Luoma
#3302 Final Project Object Detection Controller
#Simple object detection for state machine switching. 

import rospy
import cv2
import numpy as np 
from sensor_msgs.msg import Image
from sensor_msgs.msg import LaserScan
from std_msgs.msg import UInt8, UInt16
from cv_bridge import CvBridge, CvBridgeError

####GLOBAL VARIABLES####
bridge = CvBridge()
image_subscriber = None
laser_subscriber = None
stage_publisher = None
currentStage = UInt16(0)
color_ranges = []

###ROS CALLBACKS####
#img_callback reads in an RGB image, filters out target color
#Only reads and filters image during stage 1
#If object is found, go to state 2.  If no object found, stage 1 will terminate once next to table and go back to stage 0
def img_callback(data):
	global bridge 
	global currentStage
	global stage_publisher
	if currentStage == 1:
		stage_publisher.publish(UInt16(6))

		try:
			cv_image = bridge.imgmsg_to_cv2(data, "passthrough")
			# cv2.imshow('', cv_image)
			# cv2.waitKey(1)
		except CvBridgeError as e:
			print(e)
		mask = do_color_filtering(cv_image)
		blobs = get_blobs(mask)
		#Could be prone to false positives if background wasn't gray
		if len(blobs) > 0:
			stage_publisher.publish(UInt16(2))
		else:
			stage_publisher.publish(UInt16(0))

#stage_callback constantly monitors what the current state is
#Movement will control all state switches except 1 -> 2 and 3 -> 4
def stage_callback(data):
	global currentStage
	currentStage = data.data
	print(currentStage)

####IMAGE FILTERING####
#add_color_range_to_detect is used to add a new range of colors (in BGR format)
def add_color_range_to_detect(lower_bound, upper_bound):
	global color_ranges
	color_ranges.append([lower_bound, upper_bound]) # Add color range to global list of color ranges to detect

#check_if_color_in_range is used to check if a specific pixel is within any of the color ranges
#If in range, return true, else false
def check_if_color_in_range(bgr_tuple):
	# if (bgr_tuple[0] != 102):
	# 	print(bgr_tuple)
	global color_ranges
	for entry in color_ranges:
		lower, upper = entry[0], entry[1]
		in_range = True
		for i in range(len(bgr_tuple)):
			if bgr_tuple[i] < lower[i] or bgr_tuple[i] > upper[i]:
				in_range = False
				break
		if in_range: return True
	return False

#do_color_filtering takes a cv2 imgage and uses check_if_color_in_range on every pixel
#All that are in the different ranges will be set to 1, the rest will be set to 0
def do_color_filtering(img):
	img_height = img.shape[0]
	img_width = img.shape[1]
	mask = np.zeros([img_height, img_width]) # Index mask as [height, width] (e.g.,: mask[y,x])
	for i in range(img_height):
		for j in range(img_width):
			if(check_if_color_in_range(img[i][j])):
				mask[i][j] = 1
	return mask

#expand is a non recursive function that will find all pixels within range that are adjacent 
def expand(mask, current, coords_in_blob):
	coord_list = [current]
	while len(coord_list) > 0:
		current_coord = coord_list.pop()
		if not current_coord[0] in range(0,len(mask)) or not current_coord[1] in range(0, len(mask[0])):
			#print("Out of bounds")
			temp = None
		elif not mask[current_coord[0]][current_coord[1]]:
			#print("Already counted or not in blob")
			temp = None
		else:
			mask[current_coord[0]][current_coord[1]] = 0
			coords_in_blob.append(current_coord)
			coord_list.append((current_coord[0]+1, current_coord[1]))
			coord_list.append((current_coord[0]-1, current_coord[1]))
			coord_list.append((current_coord[0], current_coord[1]+1))
			coord_list.append((current_coord[0], current_coord[1]-1))
	return coords_in_blob

#once all adjacent detected pixels are found, get_blobs will return how many sets of detected pixels exist
def get_blobs(mask):
	duplicate = mask.copy()
	blobs = []
	for i in range(len(mask)):
		for j in range(len(mask[0])):
			if duplicate[i][j] == 1:
				temp = []
				expand(duplicate, (i,j), temp)
				blobs.append(temp)
	return blobs

#get_blob_centroids averages out the x and y values of every pixel in the blob to produce the coordinate of the center
#this function was used for original tests but not implemented in final design
def get_blob_centroids(blobs):
	positions = []
	for blob in blobs:
		x = []
		y = []
		for val in blob:
			x.append(val[1])
			y.append(val[0])
		positions.append((np.mean(y),np.mean(x)))
	return positions

#main function used for initializing subscribers/publishers and spinning ROS to constantly call out callbacks
def main():
	global stage_publisher
	global image_subscriber, laser_subscriber
	# add_color_range_to_detect([0,0,110], [10,10,255]) # Detect red
	# add_color_range_to_detect([0,110,0], [10,255,10]) # Detect green
	add_color_range_to_detect([110,0,0], [255,10,10]) # Detect blue
	rospy.init_node('object_detection', anonymous=True)
	image_subscriber = rospy.Subscriber('/head_camera/rgb/image_raw', Image, img_callback)
	stage_subscriber = rospy.Subscriber('/state', UInt16, stage_callback)
	stage_publisher = rospy.Publisher('/state', UInt16, queue_size=10)
	try:
		rospy.spin()
	except KeyboardInterrupt:
		print("Shutting down")
	cv2.destroyAllWindows()

if __name__ == '__main__':
	main()
