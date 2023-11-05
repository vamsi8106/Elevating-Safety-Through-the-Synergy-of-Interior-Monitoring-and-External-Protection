#https://pysource.com
import cv2
from realsense_camera import *
from mask_rcnn import *
from std_msgs.msg import Int16
import signal

class depth_est:
	def __init__(self):
		self.pub = rospy.Publisher("/depth", Int16, queue_size=10)
		# Load Realsense camera
		rs = RealsenseCamera()
		mrcnn = MaskRCNN()
		signal.signal(signal.SIGINT, self.cleanup)
		try:
			while True:
				# Get frame in real time from Realsense camera
				ret, bgr_frame, depth_frame = rs.get_frame_stream()

				# Get object mask
				boxes, classes, contours, centers = mrcnn.detect_objects_mask(bgr_frame)

				# Draw object mask
				bgr_frame = mrcnn.draw_object_mask(bgr_frame)

				# Show depth info of the objects
				temp=mrcnn.draw_object_info(bgr_frame, depth_frame)
				# print("Depth: ",temp[1])
				msg=Int16()
				msg.data=temp[1]
				self.pub.publish(msg)
				cv2.imshow("depth frame", depth_frame)
				cv2.imshow("Bgr frame", bgr_frame)

				key = cv2.waitKey(1)
				if key == 27:
					break
		except KeyboardInterrupt:
			pass

		
	def cleanup(self, signal, frame):
		print("Cleaning up and exiting...")
		self.cap.release()
		cv2.destroyAllWindows()
		exit(0)

if __name__ == "__main__":
    try:
        rospy.init_node("depth_est")
        depth_est()
        rospy.spin()
    except KeyboardInterrupt:
        pass

