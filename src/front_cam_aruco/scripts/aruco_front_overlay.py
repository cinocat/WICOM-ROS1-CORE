#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy, tf2_ros, numpy as np, cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import String

class ArucoFrontOverlay:
    def __init__(self):
        rospy.init_node('aruco_front_overlay', anonymous=False)
        self.camera_frame = rospy.get_param('~camera_frame','front_cam/usb_cam_link')
        self.marker_ids   = rospy.get_param('~marker_ids',[0,1,2,3])
        self.axis_len     = rospy.get_param('~axis_len',0.05)
        self.publish_rate = rospy.get_param('~publish_rate',15)
        self.image_topic  = rospy.get_param('~image_topic','/usb_camera/image_raw')
        self.caminfo_topic= rospy.get_param('~camera_info_topic','/usb_camera/camera_info')

        self.bridge= CvBridge()
        self.buffer= tf2_ros.Buffer(cache_time=rospy.Duration(5.0))
        self.listener= tf2_ros.TransformListener(self.buffer)

        self.image_sub= rospy.Subscriber(self.image_topic, Image, self.image_cb, queue_size=1)
        self.info_sub = rospy.Subscriber(self.caminfo_topic, CameraInfo, self.info_cb, queue_size=1)
        self.image_pub= rospy.Publisher('/front_cam/usb_cam/image_aruco_overlay', Image, queue_size=1)
        self.ids_pub  = rospy.Publisher('~visible_ids', String, queue_size=1)

        self.camera_matrix=None
        self.latest_image=None
        rospy.loginfo("[aruco_front_overlay] Started.")

    def info_cb(self,msg):
        self.camera_matrix=np.array(msg.K).reshape(3,3) if msg.K else None

    def image_cb(self,msg):
        self.latest_image=msg

    def project_point(self,p):
        if self.camera_matrix is None: return None
        X,Y,Z=p
        if Z<=0: return None
        u=(self.camera_matrix[0,0]*X + self.camera_matrix[0,2]*Z)/Z
        v=(self.camera_matrix[1,1]*Y + self.camera_matrix[1,2]*Z)/Z
        return int(round(u)), int(round(v))

    @staticmethod
    def quat_to_rot_matrix(qx,qy,qz,qw):
        R=np.zeros((3,3))
        R[0,0]=1-2*(qy**2+qz**2); R[0,1]=2*(qx*qy-qz*qw); R[0,2]=2*(qx*qz+qy*qw)
        R[1,0]=2*(qx*qy+qz*qw); R[1,1]=1-2*(qx**2+qz**2); R[1,2]=2*(qy*qz-qx*qw)
        R[2,0]=2*(qx*qz-qy*qw); R[2,1]=2*(qy*qz+qx*qw); R[2,2]=1-2*(qx**2+qy**2)
        return R

    def draw_axes(self,img,origin,R):
        axes={'x':(self.axis_len,0,0),'y':(0,self.axis_len,0),'z':(0,0,self.axis_len)}
        o2d=self.project_point(origin)
        if not o2d: return
        ox,oy=o2d
        colors={'x':(0,0,255),'y':(0,255,0),'z':(255,0,0)}
        for a,v in axes.items():
            end3d=origin + R.dot(np.array(v))
            e2d=self.project_point(end3d)
            if e2d:
                ex,ey=e2d
                cv2.line(img,(ox,oy),(ex,ey),colors[a],2)
                cv2.putText(img,a,(ex,ey),cv2.FONT_HERSHEY_SIMPLEX,0.45,colors[a],1,cv2.LINE_AA)

    def process(self):
        if self.latest_image is None or self.camera_matrix is None:
            return
        img=self.bridge.imgmsg_to_cv2(self.latest_image,'bgr8')
        visible=[]
        for mid in self.marker_ids:
            frame=f"aruco_marker_{mid}"
            try:
                trans=self.buffer.lookup_transform(self.camera_frame, frame, rospy.Time(0), timeout=rospy.Duration(0.05))
                t=trans.transform.translation
                origin=np.array([t.x,t.y,t.z])
                q=trans.transform.rotation
                R=self.quat_to_rot_matrix(q.x,q.y,q.z,q.w)
                self.draw_axes(img,origin,R)
                p2d=self.project_point(origin)
                if p2d:
                    cv2.putText(img,f"ID {mid}",p2d,cv2.FONT_HERSHEY_SIMPLEX,0.45,(255,255,255),1,cv2.LINE_AA)
                visible.append(str(mid))
            except (tf2_ros.LookupException, tf2_ros.ExtrapolationException, tf2_ros.ConnectivityException):
                continue
        if visible:
            self.ids_pub.publish(",".join(visible))
        out=self.bridge.cv2_to_imgmsg(img,encoding='bgr8')
        out.header.stamp=rospy.Time.now()
        out.header.frame_id=self.camera_frame
        self.image_pub.publish(out)

    def spin(self):
        rate=rospy.Rate(self.publish_rate)
        while not rospy.is_shutdown():
            self.process()
            rate.sleep()

if __name__=="__main__":
    ArucoFrontOverlay().spin()