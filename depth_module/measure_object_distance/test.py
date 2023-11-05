import pyrealsense2 as rs
import numpy as np
import cv2


def main(args=None):

    img_size=(640, 480)  # inference size (width, height) (x, y)

    # Configure depth and color streams
    pipeline = rs.pipeline()
    config = rs.config()

    config.enable_stream(rs.stream.depth, img_size[0], img_size[1], rs.format.z16, 30)
    config.enable_stream(rs.stream.color, img_size[0], img_size[1], rs.format.bgr8, 30)

    # Start streaming
    profile = pipeline.start(config)


    # Create an align object
    # rs.align allows us to perform alignment of depth frames to others frames
    # The "align_to" is the stream type to which we plan to align depth frames.
    align = rs.align(rs.stream.color)

    for x in range(5):
        pipeline.wait_for_frames()

    try:
        while True:

            # Wait for a coherent pair of frames: depth and color
            frames = pipeline.wait_for_frames()

            # Align the depth frame to color frame
            aligned_frames = align.process(frames)

            # Get aligned frames
            depth_frame = aligned_frames.get_depth_frame() # aligned_depth_frame is a 640x480 depth image
            color_frame = aligned_frames.get_color_frame()

            # Validate that both frames are valid
            if not depth_frame or not color_frame:
                print('\n-------Unable to get frames-------\n')

            # Convert images to numpy arrays
            depth_image = np.asanyarray(depth_frame.get_data())
            color_image = np.asanyarray(color_frame.get_data())
           

            # Apply colormap on depth image (image must be converted to 8-bit per pixel first)
            depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)

       
            disp_images = np.hstack((color_image, depth_colormap))

            cv2.imshow('RealSense', disp_images)
            # the 'esc' button is set as the quitting button
            if cv2.waitKey(1) & 0xFF == 27:
                break
    finally:
        # Stop streaming
        pipeline.stop()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()