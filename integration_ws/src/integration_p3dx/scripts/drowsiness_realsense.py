import cv2
import numpy as np
from keras.models import load_model
from keras.preprocessing.image import img_to_array
from playsound import playsound
from threading import Thread
import pyrealsense2 as rs

# Load the Drowsiness Detection Model
model = load_model(r"C:\Users\Vamsi\Desktop\Unstop\driver_drowsiness_system_CNN\drowiness_new7.h5")

# Initialize RealSense Camera
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
pipeline.start(config)

# Load Haar Cascade Classifiers for Face and Eyes
face_cascade = cv2.CascadeClassifier(r"C:\Users\Vamsi\Desktop\Unstop\driver_drowsiness_system_CNN\data\haarcascade_frontalface_default.xml")
left_eye_cascade = cv2.CascadeClassifier(r"C:\Users\Vamsi\Desktop\Unstop\driver_drowsiness_system_CNN\data\haarcascade_lefteye_2splits.xml")
right_eye_cascade = cv2.CascadeClassifier(r"C:\Users\Vamsi\Desktop\Unstop\driver_drowsiness_system_CNN\data\haarcascade_righteye_2splits.xml")

# Initialize Drowsiness Detection Variables
count = 0
alarm_on = False
alarm_sound = r"C:\Users\Vamsi\Desktop\Unstop\driver_drowsiness_system_CNN\data\sound.mp3"
classes = ['Closed', 'Open']
status1 = ''
status2 = ''

def start_alarm(sound):
    """Play the alarm sound"""
    playsound(sound)

try:
    while True:
        # Wait for RealSense frames
        frames = pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()

        if not color_frame:
            continue

        # Convert RealSense color frame to a numpy array
        color_image = np.asanyarray(color_frame.get_data())

        # Process the frame for drowsiness detection
        gray = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)
        faces = face_cascade.detectMultiScale(gray, 1.3, 5)

        # Initialize status1 and status2 here
        status1 = ''
        status2 = ''

        for (x, y, w, h) in faces:
            cv2.rectangle(color_image, (x, y), (x + w, y + h), (255, 0, 0), 1)
            roi_gray = gray[y:y + h, x:x + w]
            roi_color = color_image[y:y + h, x:x + w]

            left_eye = left_eye_cascade.detectMultiScale(roi_gray)
            right_eye = right_eye_cascade.detectMultiScale(roi_gray)

            for (x1, y1, w1, h1) in left_eye:
                # Process the left eye
                cv2.rectangle(roi_color, (x1, y1), (x1 + w1, y1 + h1), (0, 255, 0), 1)
                eye1 = roi_color[y1:y1 + h1, x1:x1 + w1]
                eye1 = cv2.resize(eye1, (145, 145))
                eye1 = eye1.astype('float') / 255.0
                eye1 = img_to_array(eye1)
                eye1 = np.expand_dims(eye1, axis=0)
                pred1 = model.predict(eye1)
                status1 = np.argmax(pred1)
                break

            for (x2, y2, w2, h2) in right_eye:
                # Process the right eye
                cv2.rectangle(roi_color, (x2, y2), (x2 + w2, y2 + h2), (0, 255, 0), 1)
                eye2 = roi_color[y2:y2 + h2, x2:x2 + w2]
                eye2 = cv2.resize(eye2, (145, 145))
                eye2 = eye2.astype('float') / 255.0
                eye2 = img_to_array(eye2)
                eye2 = np.expand_dims(eye2, axis=0)
                pred2 = model.predict(eye2)
                status2 = np.argmax(pred2)
                break

            # If the eyes are closed, start counting
            if status1 == 1 and status2 == 1:
                count += 1
                cv2.putText(color_image, "Eyes Closed, Frame count: " + str(count), (10, 30), cv2.FONT_HERSHEY_COMPLEX, 1, (0, 0, 255), 1)
                # if eyes are closed for 10 consecutive frames, start the alarm
                if count >= 10:
                    cv2.putText(color_image, "Drowsiness Alert!!!", (100, color_image.shape[0] - 20), cv2.FONT_HERSHEY_COMPLEX, 1, (0, 0, 255), 2)
                    if not alarm_on:
                        alarm_on = True
                        # play the alarm sound in a new thread
                        t = Thread(target=start_alarm, args=(alarm_sound,))
                        t.daemon = True
                        t.start()
            else:
                cv2.putText(color_image, "Eyes Open", (10, 30), cv2.FONT_HERSHEY_COMPLEX, 1, (0, 255, 0), 1)
                count = 0
                alarm_on = False

        # Display the RealSense image
        cv2.imshow('Drowsiness Detector', color_image)

        # Exit if ESC key is pressed
        if cv2.waitKey(1) == 27:
            break

finally:
    # Stop streaming and close OpenCV windows
    pipeline.stop()
    cv2.destroyAllWindows()
