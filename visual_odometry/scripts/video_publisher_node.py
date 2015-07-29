#!/usr/bin/env python
#import roslib
#roslib.load_manifest('my_package')
import sys
import rospy
import cv2

from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError





def image_sequence_pub():
    
    image_pub = rospy.Publisher("/camera/image_raw",Image,queue_size=10)

    filename = rospy.get_param('filename','bottom_camera.ogv')
    #rospy.loginfo('path =%s',filename)
    print '------------------------\n'
    print '%s' % (filename)
    # ex path ref ~/Desktop/5410502965/images/Image_ref.bmp
    
    cap = cv2.VideoCapture(filename)
    if not cap.isOpened():
        print "-----------Cannot Open file--------------\n"

    #CV_CAP_PROP_FPS
    fps = cap.get(5) #getting fps there are no variable define like c++
    print ("FrameRate of videofile =  %d" % fps)
    
    bridge = CvBridge()
    framecnt = 1
    fps = 40.00
    rate = rospy.Rate(fps) 
    down_sampling = 15.00 # 10hz
    lasttime = rospy.Time.now()

    while not rospy.is_shutdown():
        ret, frame = cap.read()
        now = rospy.Time.now()

        if not ret:
            print ("---- Video Complete ----\n" + "frametotal : %d" % framecnt + '\n')
            break        
        cv_image = frame #will down sample the rate later
        try:
            
            if now.to_sec() - lasttime.to_sec() >= (1.00/down_sampling):
                image_message = bridge.cv2_to_imgmsg(cv_image, "bgr8")
                image_pub.publish(image_message)
                framecnt = framecnt + 1
                rospy.loginfo("Publish current Frame : " + str(framecnt))
                lasttime = now
        except CvBridgeError,e:
            print e
        
        #cv2.imshow("Current Image from file" + filename ,cv_image)
        #cv2.waitKey(3)
        
        rate.sleep()



def main(args):
    rospy.init_node('image_converter', anonymous=True)
    try:
        image_sequence_pub()
    except KeyboardInterrupt:
        print "Exiting Image Publisher Node"
        cv2.destroyAllWindows()
        cap.release()

if __name__ == '__main__':
    main(sys.argv)
