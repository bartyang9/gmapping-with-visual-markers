#!/usr/bin/env python
import roslib; roslib.load_manifest('semantic_label_publisher') 
import numpy as np
import rospy
import sys
from utils import interpret_output, iou
# from yolo_publisher.msg import ObjLabel
sys.path.append("/home/ji/code/clCaffe/python")
import caffe
caffe.set_mode_cpu()
caffe.set_device(0)
# caffe.set_mode_cpu()
from datetime import datetime
import numpy as np
import getopt
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

class ObjectLabel(object):
	def __init__(self,lname,lid,lcolor):
		self.label_name  = lname
		self.label_id    = int(lid)
		self.label_color = [int(lcolor[0]),int(lcolor[1]),int(lcolor[2])]

class ObjLabelPub():
	def __init__(self,model_filename,weight_filename):
		# self.pub       = rospy.Publisher('object_label',ObjLabel)
		self.image_pub = rospy.Publisher("obj_label_image",Image)
		self.image_topic = rospy.get_param('~image_topic','camera/rgb/image_raw_throttle')
		self.image_sub   = rospy.Subscriber(self.image_topic,Image,self.image_callback)
		self.bridge = CvBridge()
		self.net         = caffe.Net(model_filename, weight_filename, caffe.TEST)
		self.transformer = caffe.io.Transformer({'data': self.net.blobs['data'].data.shape})
		self.transformer.set_transpose('data', (2,0,1))   
		# self.msg = ObjLabel()
	
	def image_callback(self, inputs):
		print 'wawa, new img!!!'
		try:
			cv_img = self.bridge.imgmsg_to_cv2(inputs, "bgr8")
			cv_img = cv2.resize(cv_img,(227,227))
	  		print 'image received'
		except CvBridgeError, e:
	  		print e
		out = self.net.forward_all(data=np.asarray([self.transformer.preprocess('data', cv_img)]))
		print out
		img_cv = cv2.cvtColor(cv_img, cv2.COLOR_RGB2BGR)
		results = interpret_output(out['result'][0], img_cv.shape[1], img_cv.shape[0])
		print results
		self.show_results(img_cv,results, img_cv.shape[1], img_cv.shape[0])

	def show_results(self, img, results, img_width, img_height):
		img_cp = img.copy()
		for i in range(len(results)):
			x = int(results[i][1])
			y = int(results[i][2])
			w = int(results[i][3])//2
			h = int(results[i][4])//2
			xmin = x-w
			xmax = x+w
			ymin = y-h
			ymax = y+h
			if xmin<0:
				xmin = 0
			if ymin<0:
				ymin = 0
			if xmax>img_width:
				xmax = img_width
			if ymax>img_height:
				ymax = img_height
			cv2.rectangle(img_cp,(xmin,ymin),(xmax,ymax),(0,255,0),2)
			print xmin, ymin, xmax, ymax
			cv2.rectangle(img_cp,(xmin,ymin-20),(xmax,ymin),(125,125,125),-1)
			cv2.putText(img_cp,results[i][0] + ' : %.2f' % results[i][5],(xmin+5,ymin-7),cv2.FONT_HERSHEY_SIMPLEX,0.5,(0,0,0),1)	
		img_cp = cv2.resize(img_cp,(1280,960))
		try:
			self.image_pub.publish(self.bridge.cv2_to_imgmsg(img_cp, "bgr8"))
		except CvBridgeError, e:
			print e

def main(args):
  rospy.init_node('obj_label_pub')
#   caffe_root      = rospy.get_param('~caffe_root', '~/caffe/')
  MODEL_FILE      = rospy.get_param('~MODEL_FILE_PATH', '~/deploy.prototxt')
  PRETRAINED      = rospy.get_param('~PRETRAINED_PATH', '~/model.caffemodel')
#   MEAN_FILE       = rospy.get_param('~MEAN_FILE_PATH', '~/mean.npy')
#   SVM_PICKLE_FILE = rospy.get_param('~SVM_PICKLE_FILE_PATH', '~/clf.pkl')
#   SUB_CAT_FILE    = rospy.get_param('~SUB_CAT_FILE','~/my_cats.txt')
  try:
	ne = ObjLabelPub(MODEL_FILE,PRETRAINED) 
  except rospy.ROSInterruptException: pass
  rospy.spin()
  
if __name__ == '__main__':
  main(sys.argv)

