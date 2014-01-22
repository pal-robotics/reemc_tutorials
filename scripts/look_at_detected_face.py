#!/usr/bin/env python

import roslib; roslib.load_manifest('reemc_tutorials')
import rospy
import smach
import math
import smach_ros
import actionlib
import pal_detection_msgs.msg
#from pal_detection_msgs.msg import FaceDetections
import sensor_msgs.msg 
import geometry_msgs.msg 
import pr2_controllers_msgs.msg



# define state Foo
class GetFaceDetectionState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['success'],
                                output_keys=['last_face_detected'])
        self.sub = rospy.Subscriber(name="/pal_face/faces", 
                                    data_class=pal_detection_msgs.msg.FaceDetections, 
                                    callback=self.face_callback)
        self.last_face_detected = None
    def face_callback(self, msg):
        #rospy.loginfo("Received %d detected faces", len(msg.faces))
        if len(msg.faces) > 0:
            self.last_face_detected = msg.faces[0]
        
    def execute(self, userdata):
        self.last_face_detected = None
        rospy.loginfo('Waiting for face detection')
        while self.last_face_detected == None:
            rospy.sleep(0.1)

        userdata.last_face_detected = self.last_face_detected

        return 'success'
        
        
        
class LookToPointState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['success', 'failed'],
                                input_keys=['last_face_detected'])
        self.camera_info = None
        self.sub = rospy.Subscriber(name="/stereo/left/camera_info", 
                                    data_class=sensor_msgs.msg.CameraInfo, 
                                    callback=self.camera_info_callback)
        self.client = actionlib.SimpleActionClient('/head_traj_controller/point_head_action',
                                                   pr2_controllers_msgs.msg.PointHeadAction)


    #Obtain the camera instrinsic parameters        
    def camera_info_callback(self, msg):
        self.camera_info = msg
    
    
    
    def execute(self, userdata):
        if self.camera_info == None:
            rospy.logerr("Camera intrinsic parameters not received")
            return 'failed'
        
        targetX = userdata.last_face_detected.x + userdata.last_face_detected.width/2
        targetY = userdata.last_face_detected.y + userdata.last_face_detected.height/2
            
    
        #K[0] is fx
        #K[2] is cx
        #K[4] is fy    
        #K[5] is cy
    
        x = (targetX - self.camera_info.K[2])/self.camera_info.K[0]
        y = (targetY - self.camera_info.K[5])/self.camera_info.K[4]
        z = 1 #Arbitrary distance
        
        if math.sqrt(pow(x, 2) + pow(y, 2)) < 0.2:
            return 'success'
        pointStamped = geometry_msgs.msg.PointStamped()      
        pointStamped.header.frame_id = "/stereo_optical_frame"
        pointStamped.header.stamp    = rospy.Time.now()
        pointStamped.point.x = x * z
        pointStamped.point.y = y * z
        pointStamped.point.z = z
        
        
        
        #build the action goal
        goal = pr2_controllers_msgs.msg.PointHeadGoal()     
      #the goal consists in making the Z axis of the cameraFrame to point towards the pointStamped
        goal.pointing_frame = "/stereo_optical_frame"
        goal.pointing_axis.x = 0.0
        goal.pointing_axis.y = 0.0
        goal.pointing_axis.z = 1.0
        goal.min_duration = rospy.Duration(0.5)
        goal.max_velocity = 1.0
        goal.target = pointStamped
        
        rospy.loginfo("Sending goal %s", str(pointStamped))
        self.client.send_goal(goal)
        self.client.wait_for_result(rospy.Duration(1.0))
        
        res = self.client.get_result()
        if res == None:
            rospy.logerr("Result of executing goal is none")
            return 'failed'
        else:
            rospy.loginfo("Result of executing goal is %s", str(res))
            return 'success'
        #todo check result if res 
        
# main
def main():
    
    rospy.init_node('look_at_detected_face')
    main_sm = smach.StateMachine(outcomes=['done', 'failed', 'preempted'])
    # Create and start the introspection server
    sis = smach_ros.IntrospectionServer('look_at_detected_face', main_sm, '/LOOK_AT_DETECTED_FACE')
    
    with main_sm:
        
        smach.StateMachine.add('WAIT_FOR_FACE', GetFaceDetectionState(),
                               transitions={'success':'LOOK_AT_FACE'})
                                   
                                   
                                     
            
            
        smach.StateMachine.add('LOOK_AT_FACE', LookToPointState(),
                               transitions={'success':'WAIT_FOR_FACE','failed':'WAIT_FOR_FACE'})
        
        
    sis.start()
    main_sm.execute()
    
    #rospy.spin()
    sis.stop()


if __name__ == '__main__':
    main()

