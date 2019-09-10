#!/usr/bin/env python
from __future__ import division
import cv2
import rospy
import threading
from ar_msgs.msg import Object, PersonArray, Person
# from ar_msgs.msg import Object, ObjectArray
from std_msgs.msg import Header
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
from copy import deepcopy

import numpy as np
import math

class ObjVel:
    def __init__(self):
        rospy.init_node('densetrack_multi', anonymous=True, log_level=rospy.DEBUG)
        ## TODO: remap!
        # flowx_topic = '/df_node/flow_xy'
        flowx_topic = 'df_img'
        self.max_Object_age = rospy.get_param('~max_Object_age', 300) ## so follow for 10 seconds at 30 fps
        #I need the size of the image that was used to get the Objects topic
        camerainfo = rospy.wait_for_message('/videofiles/camera_info', CameraInfo)
        print(camerainfo.height)
        print(camerainfo.width)
        self.height = camerainfo.height
        self.width = camerainfo.width

        rospy.loginfo('Input flow xy topic being read as background image: {}'.format(flowx_topic))

        # self.hss =  rospy.Subscriber('Objects', ObjectArray, self.updateObjects)
        self.hss =  rospy.Subscriber('rois', PersonArray, self.updatePersons)
        self.cvbridge = CvBridge()
        #maybe this is a good time for a wait_for_message; to initialize this thing correctly
        rospy.logdebug('waiting for flow topics to publish first image so I can initialize this guy ')
        self.bgxd = rospy.wait_for_message(flowx_topic, Image,2)

        ##now I need to get the size of the flow topics
        bgx = self.cvbridge.imgmsg_to_cv2(self.bgxd, 'bgr8')
        self.flowheight = bgx.shape[0]
        self.flowwidth = bgx.shape[1]
        rospy.loginfo('flow dims: {} x {}'.format(self.flowwidth,self.flowheight))
        self.wr = self.flowwidth /self.width
        self.hr = self.flowheight/self.height
        rospy.loginfo('transform rates: {} x {}'.format(self.wr,self.hr))

        self.issx = rospy.Subscriber(flowx_topic, Image, self.updatebgxy)

        self.hp = rospy.Publisher('newObjects', PersonArray, queue_size=1)
        self.hpf = rospy.Publisher('newObjects_flow', PersonArray, queue_size=1)
        # self.hp = rospy.Publisher('newObjects', ObjectArray, queue_size=1)
        # self.hpf = rospy.Publisher('newObjects_flow', ObjectArray, queue_size=1)
        self.ihpx = rospy.Publisher('newObjects_imgxy', Image, queue_size=1)
        self.lock = threading.Lock()
        self.OA = PersonArray() ### this is just a list. i could initialize it to an empty list...
        # OAC = ObjectArray()
        # self.OA = OAC.Objects ### this is just a list. i could initialize it to an empty list...
        self.Objects_age = 0 ### I am going to keep a counter to know how old the information about the Object is and not try to follow it forever.
        rospy.loginfo('Objectsvel initialized. ')
        while not rospy.is_shutdown():
            rospy.spin()

    def transformObject(self,aObject):
        rospy.logdebug('old Object is:')
        rospy.logdebug(aObject)
        newObject = Object()
        ### makes sure I am greater than zero and smaller than the flow dimensions.
        newObject.roi.x_offset      = clip(aObject.roi.x_offset  *self.wr,0,self.flowwidth)
        newObject.roi.width         = clip(aObject.roi.width     *self.wr,0,self.flowwidth)
        newObject.roi.y_offset      = clip(aObject.roi.y_offset  *self.hr,0,self.flowheight)
        newObject.roi.height        = clip(aObject.roi.height    *self.hr,0,self.flowheight)
        ### need to transform pose as well
        newObject.pose.position.x   = clip(aObject.pose.position.x *self.wr,0,self.flowwidth)
        newObject.pose.position.y   = clip(aObject.pose.position.y *self.hr,0,self.flowheight)

        rospy.logdebug('new Object is:')
        rospy.logdebug(newObject)
        return newObject

    def updateObjects(self,data):
        rospy.logdebug('updateObjects called')
        #might need a lock here too...
        with self.lock:
            self.OA = data.objects
            self.Objects_age = 0 ## got new Object!
            rospy.logdebug(self.OA)
            #rospy.logwarn(self.Objects_age)


    def updatePersons(self,data):
        rospy.logdebug('updatePersons called')
        #might need a lock here too...
        with self.lock:
            self.OA = data ### personsArray!
            self.Objects_age = 0 ## got new Object!
            rospy.logdebug(self.OA)
            #rospy.logwarn(self.Objects_age)

    def updatebgxy(self,data):
        with self.lock:
            thisAge = self.Objects_age
        if thisAge < self.max_Object_age:
            bg = self.cvbridge.imgmsg_to_cv2(data, 'bgr8')
            #rospy.logwarn(bg.shape)
            with self.lock:
                myhaC = deepcopy(self.OA)
                if not myhaC:
                    return
                #rospy.logwarn(myha)
            hfC = PersonArray()
            hfC.header = Header() ### maybe I could change something to tell this is updated?
            for myha in myhaC.data:
                hf = Person()
                for i in range(0, len(myha)):
                    newObject = self.transformObject(myha[i])
                    #hf.append(newObject) ###??? I'm appending this twice?
                    rospy.logdebug(bg.shape)
                    myObjectregion = bg[int(newObject.roi.y_offset):int(newObject.roi.height), int(newObject.roi.x_offset):int(newObject.roi.width)]
                    rospy.logdebug(myObjectregion.shape)
                    #maybe I want to publish ddx?
                    if myObjectregion.shape[0]>0 and myObjectregion.shape[1]>0: #only changes Object if not size 0
                        avgs = np.average(np.average(myObjectregion,axis=0), axis=0)
                        rospy.logdebug('mean value of region {}, {}'.format(avgs[0], avgs[1]))

                        ##there is a bunch of conversions here and I am not being careful this will break if images change!
                        ddxf = 0.2*(avgs[0]-127)
                        ddyf = 0.2*(avgs[1]-127)
                        ddx = ddxf/self.wr
                        ddy = ddyf/self.hr
                        rospy.logdebug('dx {}, dy {}'.format(ddx, ddy))
                        if not math.isnan(ddx):
                            ### if I don't want to accumulate error I need to update based on the float number, so
                            ### these values should be sizes really. and floats as well,
                            ### without knowing too much about pcl stuff, I don't think I can do anything too great here. so I will hack it until it works.
                            myha[i].pose.position.x = clip(myha[i].pose.position.x - ddx, 0,self.width )
                            ###these guys should be sizes. i am just making this hard to understand by trying to force the ROIs here...
                            myha[i].roi.x_offset    = clip(myha[i].roi.x_offset    - ddx ,0,self.width )
                            myha[i].roi.width       = clip(myha[i].roi.width       - ddx ,0,self.width )
                            newObject.roi.x_offset  = clip(newObject.roi.x_offset  - ddxf,0,self.width )
                            newObject.roi.width     = clip(newObject.roi.width     - ddxf,0,self.width )
                        if not math.isnan(ddy):
                            myha[i].pose.position.y = clip(myha[i].pose.position.y - ddy, 0,self.height)
                            myha[i].roi.y_offset    = clip(myha[i].roi.y_offset    - ddy ,0,self.height)
                            myha[i].roi.height      = clip(myha[i].roi.height      - ddy ,0,self.height)
                            newObject.roi.y_offset  = clip(newObject.roi.y_offset  - ddyf,0,self.height)
                            newObject.roi.height    = clip(newObject.roi.height    - ddyf,0,self.height)
                        hf.joints.append(newObject)
                        if i == 0:

                            #pass
                            self.ihpx.publish(self.cvbridge.cv2_to_imgmsg(myObjectregion,'bgr8'))
                        else:
                            rospy.logwarn('more then one Object! Objects are still not ordered, so I will mess up!!!!')
                    #rospy.signal_shutdown('damn it')
                hfC.data.append(hf)
            #this however is not fine. probably needs a lock?
            #maybe I can get away with more speed if I write the controller to be threaded as well under pan and tilt
            self.hpf.publish(hfC)
            with self.lock:
                self.OA = myhaC
                self.hp.publish(self.OA)
        else:
            rospy.logdebug('Object age limit reached. Not publishing new Objects!')
        with self.lock:
            self.Objects_age = self.Objects_age + 1

def clip(this,minT,maxT):
    return min(max(minT,this) ,maxT)

def iclip(this,minT,maxT):
    return int(clip(this,minT,maxT))

if __name__ == '__main__':

    sf = ObjVel()
