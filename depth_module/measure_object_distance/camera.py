import pyrealsense2 as rs
import cv2
import numpy as np
# Configure depth and color streams
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

# Start streaming
pipeline.start(config)

try:
    while True:
        # Wait for a coherent pair of frames: color and depth
        frames = pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        
        if not color_frame:
            continue
        
        # Convert the color frame to a numpy array
        color_image = np.asanyarray(color_frame.get_data())

        # Render image

        images = np.hstack((color_image))
        # Display the image
        cv2.imshow('RealSense', images)

        # Exit if ESC key is pressed
        if cv2.waitKey(1) == 27:
            break

finally:
    # Stop streaming
    pipeline.stop()

# Close all OpenCV windows
cv2.destroyAllWindows()
