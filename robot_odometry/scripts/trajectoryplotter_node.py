#!/usr/bin/python
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import time
import rospy
import math
from nav_msgs.msg import Odometry
from std_msgs.msg import String
import tf
from robot_odometry.srv import *


class TrajectoryPlotter:
    # Constructor
    def __init__(self):
        self.X = [0]
        self.Y = [0]
        #self.U = [0] #Pointing GUI Right
        #self.V = [1]
        self.U = [-1] #Pointing GUI Right
        self.V = [0]
        self.axislimit = [-0.7,0.7,-0.7,0.7]
        self.numberPoints = 0
        self.fig , self.ax = plt.subplots()
        self.sc = self.ax.scatter(0,0,s = 400,color = 'red') #Initialize Reddot at start postion
        self.sc = self.ax.scatter(self.X,self.Y,s = 80, color = 'blue') #Initialize Trajectory path in blue
        self.quv = self.ax.quiver(self.X,self.Y,self.U,self.V,scale = 35) #
        self.ax.axis(self.axislimit)
        plt.title('Trajectory Plot')
        #plt.ion()
        plt.grid()
        


 

    # Print the polygon
    def update(self,_):
        try:
            self.sc.set_offsets(np.column_stack((self.X,self.Y)))
            self.quv.set_UVC(self.U,self.V)
            self.quv.set_offsets(np.column_stack((self.X,self.Y)))
        except:
            pass
        #plt.draw()
        return self.sc,

    # append a point
    def add(self,x,y,u,v):
        self.numberPoints += 1
        self.X.append(x)
        self.Y.append(y)
        self.U.append(u)
        self.V.append(v)


### end class TrajectoryPlotter        

## class for ROSNODE
class UiRunner:
    def __init__(self):
       
        self.tp = TrajectoryPlotter()
        
        self.direction = 0
        rospy.init_node('uiplotter', anonymous=True)
        self.sub = rospy.Subscriber("odom", Odometry, self.odomCallback , queue_size = 10)
        ani = animation.FuncAnimation(self.tp.fig,self.tp.update, interval=10,blit=False)
        self.schangeaxis = rospy.Service('/plotter/changeaxis', Uichangeaxis, self.uichangeaxis_handle)
        self.suiclear = rospy.Service('/plotter/uiclear', Uiclear, self.uiclear_handle)
        self.ssetstart = rospy.Service('/plotter/setstartpts', Uisetstartpoint, self.uisetstartpts_handle)
        plt.show()
        rospy.spin()
    
    def uiclear_handle(self,req):
        self.tp.numberPoints = 0
        self.tp.X = [0]
        self.tp.Y = [0]
        self.tp.U = [1]
        self.tp.V = [0]
        res = UiclearResponse()
        res.status = '---Clear Points Complete---' 
        return res

    def uisetstartpts_handle(self,req):
        self.tp.X = [req.x]
        self.tp.y = [req.y]
        angledirecrad = req.theta*(math.pi/180.0)
        self.tp.U = [np.cos(angledirecrad)]
        self.tp.V = [np.sin(angledirecrad)]
        rest = Uisetstartpoint()
        rest.status = 'OK'
        return rest




    
    def uichangeaxis_handle(self,req):
        self.tp.axislimit[0] = req.xmin
        self.tp.axislimit[1] = req.xmax
        self.tp.axislimit[2] = req.ymin
        self.tp.axislimit[3] = req.ymax
        self.tp.ax.axis(self.tp.axislimit)
        st = 'Current axislimit : ' + 'x = [' + str(float(self.tp.axislimit[0])) + ' ' + str(float(self.tp.axislimit[1])) + ']' 'y = [' + str(float(self.tp.axislimit[2])) + ' ' + str(float(self.tp.axislimit[3])) + ']'
        res = UichangeaxisResponse()
        res.status = st
        return res


    def odomCallback(self,data):
        x = data.pose.pose.position.x
        y = data.pose.pose.position.y
        (r,p,th) = tf.transformations.euler_from_quaternion([data.pose.pose.orientation.x,data.pose.pose.orientation.y,data.pose.pose.orientation.z,data.pose.pose.orientation.w])
        uth = np.cos(th)
        vth = np.sin(th)
        self.tp.add(x,y,uth,vth)
        cout = "Update : X = %f , Y = %f , th = %f \n" % (x,y,th)
        rospy.loginfo(cout)



        

if __name__ == '__main__':
    try:
        ui = UiRunner()
    except rospy.ROSInterruptException: pass


        
        
