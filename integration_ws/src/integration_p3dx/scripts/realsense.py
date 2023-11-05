from realsense_camera import *
import cv2
from mask_rcnn import *

#load the realsense camera
rs=RealsenseCamera()
mrcnn=MaskRCNN()

while True:
    ret,bgr_frame,depth_frame=rs.get_frame_stream()

    boxes,classes,conyours,centers = mrcnn.detect_objects_mask(bgr_frame)

    bgr_frame=mrcnn.draw_object_mask(bgr_frame)

    mrcnn.draw_object_info(bgr_frame,depth_frame)
    

    cv2.imshow('BGR Frame',bgr_frame)
    key=cv2.waitKey(1)
    if key==27:
        break
