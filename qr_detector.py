# !/usr/bin python
import sys
# print(sys.version)
# sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages')
from threading import Thread
# import pyrealsense2 as rs
import numpy as np
import cv2
from pyzbar import pyzbar
from geometry_msgs import msg
import rospy
import sys
import copy
import tf
from sensor_msgs.msg import Image
from PIL import Image as IM
from cv_bridge import CvBridge
from std_msgs.msg import String
from detect_qr.msg import qrmsg

# Configure depth and color streams
# pipeline = rs.pipeline()
# config = rs.config()

# # Get device product line for setting a supporting resolution
# pipeline_wrapper = rs.pipeline_wrapper(pipeline)
# pipeline_profile = config.resolve(pipeline_wrapper)
# device = pipeline_profile.get_device()
# device_product_line = str(device.get_info(rs.camera_info.product_line))
class detect_qrcode(object):
	def __init__(self):
		self.image = None
		self.tr_pos = None
		self.bl_pos = None
		self.camera_topic = "/camera/color/image_raw"
		self.subscriber()
		self.pub = rospy.Publisher("qr_image", Image, queue_size=10)
		self.pub_pos = rospy.Publisher("qr_pos", qrmsg, queue_size=10 )
		self.bridge = CvBridge()

	def subscriber(self):
		rospy.Subscriber(self.camera_topic, Image, self.image_callback,queue_size =1, buff_size= 2**24)

	def image_callback(self,data):
		data = self.bridge.imgmsg_to_cv2(data, desired_encoding='passthrough')
		data = cv2.cvtColor(data, cv2.COLOR_BGR2RGB)
		self.image = data
		

	def find_qr(self):
		img = self.image
		barcodes = pyzbar.decode(img)
		# print(barcodes)
		for barcode in barcodes:
			# extract the bounding box location of the barcode and draw
			# the bounding box surrounding the barcode on the image
			print(barcode.data)
			(x, y, w, h) = barcode.rect
			if barcode.data == 'TOP_RIGHT':
				self.tr_pos = [x, y, w, h]
				print(self.tr_pos)
			elif barcode.data == 'BOTTOM_LEFT':
				self.bl_pos = [x, y, w, h]
			cv2.rectangle(img, (x, y), (x + w, y + h), (0, 0, 255), 2)
			# the barcode data is a bytes object so if we want to draw it
			# on our output image we need to convert it to a string first
			barcodeData = barcode.data.decode("utf-8")
			barcodeType = barcode.type
			# draw the barcode data and barcode type on the image
			text = "{} ({})".format(barcodeData, barcodeType)
			cv2.putText(img, text, (x, y - 10),
				cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
		return img
	
	def publish_img(self,data):
		image = self.bridge.cv2_to_imgmsg(data, encoding="passthrough")
		self.pub.publish(image)
		pass
	
	def publish_qr(self):
		if self.tr_pos is not None and self.bl_pos is not None:
		# if self.tr_pos[0] != 0:
			print(self.tr_pos)
			qrpos = [0, 0, 0, 0, 0, 0, 0, 0]
			qrpos[0:4] = self.tr_pos
			qrpos[4:8] = self.bl_pos
			# qrpos= [self.tr_pos,[self.bl_pos]]
			# qrpos = qrpos.extend(self.tr_pos)
			# qrpos = qrpos.extend(self.bl_pos)
			# qrpos.append(self.tr_pos)
			# qrpos.append(self.bl_pos)
			print(qrpos)
			self.pub_pos.publish(qrpos)
		else:
			pass

if __name__=="__main__":
	rospy.init_node("Detect_Qr")
	qr = detect_qrcode()
	# cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
	# cv2.imshow('RealSense', images)
	# key = cv2.waitKey(1)

		# if the `q` key was pressed, break from the loop
	# if key == ord("q"):
	# 	print("[INFO] Shutting down...")
	# 	cv2.destroyAllWindows()
	# 	break
	while not rospy.is_shutdown():
		if qr.image is not None:
			image = qr.find_qr()
			qr.publish_img(image)
			qr.publish_qr()









"""
# import the necessary packages
from imutils.video import VideoStream
from pyzbar import pyzbar
import argparse
import datetime
import imutils
import time
import cv2

# construct the argument parser and parse the arguments
ap = argparse.ArgumentParser()
ap.add_argument("-o", "--output", type=str, default="barcodes.csv",
	help="path to output CSV file containing barcodes")
args = vars(ap.parse_args())

# initialize the video stream and allow the camera sensor to warm up
print("[INFO] starting video stream...")
vs = VideoStream(src=0).start()
# vs = VideoStream(usePiCamera=False).start()
time.sleep(2.0)

# open the output CSV file for writing and initialize the set of
# barcodes found thus far
csv = open(args["output"], "w")
found = set()

# loop over the frames from the video stream
while True:
	# grab the frame from the threaded video stream and resize it to
	# have a maximum width of 400 pixels
	frame = vs.read()
	frame = imutils.resize(frame, width=800)
	# find the barcodes in the frame and decode each of the barcodes
	barcodes = pyzbar.decode(frame)

		# loop over the detected barcodes
	for barcode in barcodes:
		# extract the bounding box location of the barcode and draw
		# the bounding box surrounding the barcode on the image
		(x, y, w, h) = barcode.rect
		cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 0, 255), 2)
		# the barcode data is a bytes object so if we want to draw it
		# on our output image we need to convert it to a string first
		barcodeData = barcode.data.decode("utf-8")
		barcodeType = barcode.type
		# draw the barcode data and barcode type on the image
		text = "{} ({})".format(barcodeData, barcodeType)
		cv2.putText(frame, text, (x, y - 10),
			cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
		# if the barcode text is currently not in our CSV file, write
		# the timestamp + barcode to disk and update the set
		if barcodeData not in found:
			csv.write("{},{}\n".format(datetime.datetime.now(),
				barcodeData))
			csv.flush()
			found.add(barcodeData)

	# show the output frame
	cv2.imshow("Barcode Scanner", frame)
	key = cv2.waitKey(1) & 0xFF

	# if the `q` key was pressed, break from the loop
	if key == ord("q"):
		# close the output CSV file do a bit of cleanup
		print("[INFO] cleaning up...")
		csv.close()
		cv2.destroyAllWindows()
		vs.stop()
		break
		
"""
