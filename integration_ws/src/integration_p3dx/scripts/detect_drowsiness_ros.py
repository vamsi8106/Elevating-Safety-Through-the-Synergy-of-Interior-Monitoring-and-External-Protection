import cv2
import numpy as np
import rospy
from std_msgs.msg import Int8
from keras.models import load_model
from keras.preprocessing.image import img_to_array
from playsound import playsound
from threading import Thread
import signal

class unstop:
    def __init__(self):
        self.pub = rospy.Publisher("/drowsiness", Int8, queue_size=10)
        self.classes = ['Closed', 'Open']
        self.face_cascade = cv2.CascadeClassifier(r"/home/pc/Volkswagon/driver_drowsiness_system_CNN/data/haarcascade_frontalface_default.xml")
        self.left_eye_cascade = cv2.CascadeClassifier(r"/home/pc/Volkswagon/driver_drowsiness_system_CNN/data/haarcascade_lefteye_2splits.xml")
        self.right_eye_cascade = cv2.CascadeClassifier(r"/home/pc/Volkswagon/driver_drowsiness_system_CNN/data/haarcascade_righteye_2splits.xml")
        self.cap = cv2.VideoCapture(0)
        self.model = load_model(r"/home/pc/Volkswagon/driver_drowsiness_system_CNN/drowiness_new7.h5")
        self.count = 0
        self.alarm_on = False
        self.alarm_sound = r"/home/pc/Volkswagon/driver_drowsiness_system_CNN/data/sound.mp3"
        self.status1 = ''
        self.status2 = ''
        signal.signal(signal.SIGINT, self.cleanup)

        try:
            while True:
                _, frame = self.cap.read()
                height = frame.shape[0]
                gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                faces = self.face_cascade.detectMultiScale(gray, 1.3, 5)
                for (x, y, w, h) in faces:
                    cv2.rectangle(frame, (x, y), (x+w, y+h), (255, 0, 0), 1)
                    roi_gray = gray[y:y+h, x:x+w]
                    roi_color = frame[y:y+h, x:x+w]
                    left_eye = self.left_eye_cascade.detectMultiScale(roi_gray)
                    right_eye = self.right_eye_cascade.detectMultiScale(roi_gray)
                    for (x1, y1, w1, h1) in left_eye:
                        cv2.rectangle(roi_color, (x1, y1), (x1 + w1, y1 + h1), (0, 255, 0), 1)
                        eye1 = roi_color[y1:y1+h1, x1:x1+w1]
                        eye1 = cv2.resize(eye1, (145, 145))
                        eye1 = eye1.astype('float') / 255.0
                        eye1 = img_to_array(eye1)
                        eye1 = np.expand_dims(eye1, axis=0)
                        pred1 = self.model.predict(eye1)
                        self.status1 = np.argmax(pred1)
                        break

                    for (x2, y2, w2, h2) in right_eye:
                        cv2.rectangle(roi_color, (x2, y2), (x2 + w2, y2 + h2), (0, 255, 0), 1)
                        eye2 = roi_color[y2:y2 + h2, x2:x2 + w2]
                        eye2 = cv2.resize(eye2, (145, 145))
                        eye2 = eye2.astype('float') / 255.0
                        eye2 = img_to_array(eye2)
                        eye2 = np.expand_dims(eye2, axis=0)
                        pred2 = self.model.predict(eye2)
                        self.status2 = np.argmax(pred2)
                        break

                    # If the eyes are closed, start counting
                    if self.status1 == 2 and self.status2 == 2:
                        self.count += 1
                        cv2.putText(frame, "Eyes Closed, Frame count: " + str(self.count), (10, 30), cv2.FONT_HERSHEY_COMPLEX, 1, (0, 0, 255), 1)
                        # if eyes are closed for 10 consecutive frames, start the alarm
                        if self.count >= 2:
                            cv2.putText(frame, "Drowsiness Alert!!!", (100, height-20), cv2.FONT_HERSHEY_COMPLEX, 1, (0, 0, 255), 2)
                            if not self.alarm_on:
                                self.alarm_on = True
                                # play the alarm sound in a new thread
                                t = Thread(target=self.start_alarm, args=(self.alarm_sound,))
                                t.daemon = True
                                t.start()
                    else:
                        cv2.putText(frame, "Eyes Open", (10, 30), cv2.FONT_HERSHEY_COMPLEX, 1, (0, 255, 0), 1)
                        self.count = 0
                        self.alarm_on = False

                cv2.imshow("Drowsiness Detector", frame)

                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
        except KeyboardInterrupt:
            pass

    def start_alarm(self, sound):
        """Play the alarm sound"""
        playsound(r'/home/pc/Volkswagon/driver_drowsiness_system_CNN/data/sound.mp3')
        msg = Int8()
        msg.data = 1
        self.pub.publish(msg)

    def cleanup(self, signal, frame):
        print("Cleaning up and exiting...")
        self.cap.release()
        cv2.destroyAllWindows()
        exit(0)

if __name__ == "__main__":
    try:
        rospy.init_node("Unstop")
        unstop()
        rospy.spin()
    except KeyboardInterrupt:
        pass
