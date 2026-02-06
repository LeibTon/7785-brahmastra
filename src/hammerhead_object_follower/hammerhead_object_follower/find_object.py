import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy

import sys

import numpy as np
import cv2

class MinimalVideoSubscriber(Node):

	def __init__(self):		
		# Creates the node.
		super().__init__('minimal_video_subscriber')

		# Set Parameters
		self.declare_parameter('show_image_bool', True)
		self.declare_parameter('window_name', "Raw Image")

		#Determine Window Showing Based on Input
		self._display_image = bool(self.get_parameter('show_image_bool').value)

		# Declare some variables
		self._titleOriginal = self.get_parameter('window_name').value # Image Window Title	
		
		#Only create image frames if we are not running headless (_display_image sets this)
		if(self._display_image):
		# Set Up Image Viewing
			cv2.namedWindow(self._titleOriginal, cv2.WINDOW_AUTOSIZE ) # Viewing Window
			cv2.moveWindow(self._titleOriginal, 50, 50) # Viewing Window Original Location
			
        ## Code from find_object.py
		# CONFIG
        self.RESIZE_W = 640
	    self.RESIZE_H = 640, 480

        # Yellow
        self.LOWER = np.array([20, 100, 100])
        self.UPPER = np.array([35, 255, 255])

        self.MIN_AREA = 1200
        self.KERNEL_SIZE = 5
        self.PRINT_EVERY_N_FRAMES = 5
		
		#Set up QoS Profiles for passing images over WiFi
		image_qos_profile = QoSProfile(
		    reliability=QoSReliabilityPolicy.BEST_EFFORT,
		    history=QoSHistoryPolicy.KEEP_LAST,
		    durability=QoSDurabilityPolicy.VOLATILE,
		    depth=1
		)

		#Declare that the minimal_video_subscriber node is subcribing to the /camera/image/compressed topic.
		self._video_subscriber = self.create_subscription(
				CompressedImage,
				'/image_raw/compressed',
				self._image_callback,
				image_qos_profile)
		self._video_subscriber # Prevents unused variable warning.
	

	def _image_callback(self, CompressedImage):	
		# The "CompressedImage" is transformed to a color image in BGR space and is store in "_imgBGR"
		# Convert compressed image data to numpy array
		np_arr = np.frombuffer(CompressedImage.data, np.uint8)
		# Decode image using OpenCV
		self._imgBGR = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
		
        self._imgBGR = cv2.resize(self._imageBGR, (self.RESIZE_W, self.RESIZE_H))
        hsv = cv2.cvtColor(self._imgBGR, cv2.COLOR_BGR2HSV)

        mask = cv2.inRange(hsv, self.LOWER, self.UPPER)

        # Minimal cleanup: open removes specks, close fills small holes
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self.kernel, iterations=1)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, self.kernel, iterations=1)

        cnt = self.find_largest_contour(mask)
        found = False
        px = 0
	    py = 0
        if cnt is not None:
            area = cv2.contourArea(cnt)
            if area >= MIN_AREA:
                found = True
                cxcy = self.contour_centroid(cnt)
                if cxcy is not None:
                    cx, cy = cxcy
					px = cx - self.RESIZE_W/2
					py = cy - self.RESIZE_H/2

                    # Draw contour + bounding box + centroid
                    # cv2.drawContours(frame, [cnt], -1, (0, 255, 0), 2)
                    # x, y, w, h = cv2.boundingRect(cnt)
                    # cv2.rectangle(frame, (x, y), (x + w, y + h), (255, 0, 0), 2)
                    # cv2.circle(frame, (cx, cy), 6, (0, 0, 255), -1)
                    # cv2.putText(frame, f"({cx}, {cy})", (cx + 10, cy - 10),
                    #             cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

                    # # Print pixel coordinate
                    # if frame_count % PRINT_EVERY_N_FRAMES == 0:
                    #     print(f"Object pixel: x={cx}, y={cy}   area={int(area)}")
		
        # if not found:
        #     cv2.putText(frame, "LOST", (20, 40),
        #                 cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 2)
		
		# if(self._display_image):
		# 	# Display the image in a window
		# 	self.show_image(self._imgBGR)

	def show_image(self, img):
		cv2.imshow(self._titleOriginal, img)
		# Cause a slight delay so image is displayed
		self._user_input=cv2.waitKey(50) #Use OpenCV keystroke grabber for delay.
		
    def find_largest_contour(self, mask: np.ndarray):
        contours_info = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        contours = contours_info[0] if len(contours_info) == 2 else contours_info[1]
        if not contours:
            return None
        return max(contours, key=cv2.contourArea)

    def contour_centroid(self, cnt):
        M = cv2.moments(cnt)
        if M["m00"] == 0:
            return None
        cx = int(M["m10"] / M["m00"])
        cy = int(M["m01"] / M["m00"])
        return cx, cy

	def get_user_input(self):
		return self._user_input


def main():
	rclpy.init() #init routine needed for ROS2.
	video_subscriber = MinimalVideoSubscriber() #Create class object to be used.

	while rclpy.ok():
		rclpy.spin_once(video_subscriber) # Trigger callback processing.
		if(video_subscriber._display_image):	
			if video_subscriber.get_user_input() == ord('q'):
				cv2.destroyAllWindows()
				break
	rclpy.logging.get_logger("Camera Viewer Node Info...").info("Shutting Down")
	#Clean up and shutdown.
	video_subscriber.destroy_node()  
	rclpy.shutdown()


if __name__ == '__main__':
	main()
