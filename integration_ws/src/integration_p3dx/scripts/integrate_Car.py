import rospy
from std_msgs.msg import Int8,Int16
from geometry_msgs.msg import Twist

class Integration:
    def __init__(self):
        self.thresh_depth=2000
        self.pub=rospy.Publisher("/RosAria/cmd_vel",Twist,queue_size=10)
        self.sub1=rospy.Subscriber("/drowsiness",Int8,self.callback)

    def callback(self,msg):
        rospy.loginfo("Dowsiness")
        if(msg.data==1):
            rospy.loginfo("Drowsiness Detected")
            self.sub2=rospy.Subscriber("/depth",Int16,self.callback1)
        else:
            rospy.loginfo("Driver is Active")

    def callback1(self,msg1):
        rospy.loginfo("Depth Entered")
        if(msg1.data<self.thresh_depth):
            rospy.loginfo("Depth Threhsold Entered")
            msg2=Twist()
            msg2.linear.x=0
            msg2.angular.x=0
            self.pub.publish(msg2)
            
        else:
            pass      





if __name__=="__main__":
    rospy.init_node("Integration")
    Integration()
    rospy.spin()
