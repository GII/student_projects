import rospy
import rospkg
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np
import cv2 as cv
from cv_bridge import CvBridge
from std_msgs.msg import Float32MultiArray
import tensorflow as tf

class FeatureExtractor():

    def __init__(self):
        self.bridge=CvBridge()
        rp = rospkg.RosPack()
        model_path = rp.get_path('ur5e_perception') + "/results/model/encoder/"
        
        
        rospy.Subscriber("/camera/depth/image_rect_raw", Image, self._camera_depth_image_raw_callback)
        rospy.Subscriber("/camera/color/image_raw", Image, self._camera_rgb_image_raw_callback)
        self.depth_features_pub = rospy.Publisher("features/depth",Float32MultiArray,queue_size=10)
        self.color_features_pub = rospy.Publisher("features/color",Float32MultiArray,queue_size=10)
        self.depth_features_model = tf.keras.models.load_model(model_path+"depth.h5")
        self.color_features_model = tf.keras.models.load_model(model_path+"color.h5")
             



    def _extract_depth(self):
        rospy.wait_for_message("/camera/depth/image_rect_raw",Image,timeout=5.0)
        input = self.camera_depth_image_raw 
        input = np.reshape(input,(1,256,256,3))
        output = self.depth_features_model.predict(input)
        return output


    def _extract_color(self):
        rospy.wait_for_message("/camera/color/image_raw",Image,timeout=5.0)
        input = self.camera_rgb_image_raw 
        input = np.reshape(input,(1,256,256,3))
        output = self.color_features_model.predict(input)
        return output

    def publish_depth(self):

        rate = rospy.Rate(100)
        while not rospy.is_shutdown():
            features = self._extract_depth()
            msg = Float32MultiArray()  
            msg.data=features[0]          
            self.depth_features_pub.publish(msg)
            rate.sleep()

    def publish_color(self):
        rate = rospy.Rate(100)
        while not rospy.is_shutdown():
            features = self._extract_color()
            msg = Float32MultiArray()  
            msg.data=features[0]           
            self.color_features_pub.publish(msg)
            rate.sleep()


    def _camera_depth_image_raw_callback(self,data):
        cv_image=self.bridge.imgmsg_to_cv2(data,desired_encoding='passthrough')
        img_n = cv.normalize(src=cv_image, dst=None, alpha=0, beta=255, norm_type=cv.NORM_MINMAX, dtype=cv.CV_8U)
        img_n = 255 - img_n
        im_color= cv.applyColorMap(img_n, cv.COLORMAP_JET)
        width,height,channels =im_color.shape
        cropped_img = im_color[0:int(width),0:int(height/2)]
        resized_img = cv.resize(cropped_img, (256,256), interpolation = cv.INTER_AREA)
        input_image = (resized_img /255.0) 

        self.camera_depth_image_raw = input_image


    def _camera_rgb_image_raw_callback(self,data):

        image = self.bridge.imgmsg_to_cv2(data,desired_encoding="rgb8")
        width,height,channels = image.shape
        cropped_img = image[0:int(width),0:int(height/2)]
        resized_img = cv.resize(cropped_img, (256,256), interpolation = cv.INTER_AREA)
        input_image = (resized_img /255.0) 

        self.camera_rgb_image_raw = input_image  




