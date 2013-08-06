#!/usr/bin/env python

import roslib; roslib.load_manifest('reemc_tutorials')
import rospy
import smach
import smach_ros
from smach_ros import MonitorState, SimpleActionState, IntrospectionServer, ActionServerWrapper
import pal_detection_msgs
import sensor_msgs.msg
import geometry_msgs.msg 


# define state Foo
class GetFaceDetectionState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['success'])
        self.sub = rospy.Subscriber(name="/pal_face/faces", data_class=pal_detection_msgs.FaceDetections, callback=self.face_callback)
        self.last_face_detected = None
    def face_callback(self, msg):
        rospy.loginfo("Received %d detected faces", len(faces))
        if len(faces) > 0:
            self.last_face_detected = msg[0]
        
    def execute(self, userdata):
        self.last_face_detected = None
        rospy.loginfo('Waiting for face detection')
        while self.last_face_detected == None:
            rospy.sleep(0.1)

        rospy.loginfo('Face detected')
        userdata.last_face_detected = self.last_face_detected
        
        
class LookToPointState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['success', 'failed'])
        self.camera_info = None
        self.sub = rospy.Subscriber(name="/stereo/left/camera_info", data_class=sensor_msgs.CameraInfo, callback=self.camera_info_callback)


    #Obtain the camera instrinsic parameters        
    def camera_info_callback(self, msg):
        self.camera_info = msg
    
    
    
    def execute(self, userdata):
        if self.camera_info == None:
            rospy.logerr("Camera intrinsic parameters not received")
            return 'failed'
        
        targetX = userdata.last_face_detected.x + width/2
        targetY = userdata.last_face_detected.y + height/2
            
    
        #K[0] is fx
        #K[2] is cx
        #K[4] is fy    
        #K[5] is cy
    
        x = (targetX - self.camera_info.K[2])/self.camera_info.K[0]
        y = (targetY - self.camera_info.K[5])/self.camera_info.K[4]
        z = 1 #Arbitrary number
        
        
        geometry_msg
        target_point = msg.
        x = self.camera_info.K
# main
def main():
    
    rospy.init_node('look_at_detected_face')
    main_sm = smach.StateMachine(outcomes=['done','failed','preempted'])
    # Create and start the introspection server
    sis = smach_ros.IntrospectionServer('look_at_detected_face', main_sm, '/LOOK_AT_DETECTED_FACE')

    with main_sm:
        
        smach.StateMachine.add('WAIT_FOR_FACE', GetFaceDetectionState(),
                               transitions={'success':'LOOK_AT_FACE'},
                                output_keys=['foo_counter_out'])
                                   
                                   
                                     
            
            if use_dummy_navigation_components:
                throttle_done_transition="APPROACH_PERSON"
            else:
                throttle_done_transition="DISABLE_PATROL"
            
            smach.StateMachine.add('MEET_PEOPLE_THROTTLE',ThrottleState(20*1000),
                                   transitions={'done':throttle_done_transition,'failed':'DETECT_PEOPLE'})       
            
            if not use_dummy_navigation_components:
                disable_patrol = DisablePatrolState()
                smach.StateMachine.add('DISABLE_PATROL', disable_patrol, 
                                       transitions = {'succeeded':'APPROACH_PERSON','aborted':'failed'})
    
            ## APPROACH_PERSON
            # if use_dummy_navigation_components or no_approach_person is True:
            # This state is created with a dummy implementation of the states that
            # require navigation. This is to be able to test the whole state
            # machine with the PAL simulator, and without need to execute the
            # navigation services.
         	no_approach_person = True
        	use_dummy_approach = use_dummy_navigation_components or no_approach_person
            approach_person_state = create_approach_person_state(dummy_implementation = use_dummy_approach)
            smach.StateMachine.add('APPROACH_PERSON', approach_person_state,
                             transitions={'done':'PRESENT_TO_PERSON'})
            
            
            print options.loop
            if options.loop:
                last_state_transitions={'succeeded':first_state_name, 'aborted':first_state_name}
            else:  
                last_state_transitions={'succeeded':'done', 'aborted':'failed'}
            
            present_to_person = PresentToPersonState()
            smach.StateMachine.add('PRESENT_TO_PERSON', present_to_person,
                             transitions=last_state_transitions)
        
        smach.StateMachine.add('MEET_PEOPLE', core_sm,
                               transitions={'failed':'FINAL_DISABLE_PATROL','preempted':'FINAL_DISABLE_PATROL'})
        
        final_disable_patrol = DisablePatrolState()
        smach.StateMachine.add('FINAL_DISABLE_PATROL', final_disable_patrol, 
                               transitions = {'succeeded':'failed','aborted':'failed'})
        
    sis.start()



    use_action_server=True
    if use_action_server:
        ####################################################################################
        # Construct action server wrapper
        ####################################################################################
        asw = ActionServerWrapper("meet_people_action", meet_peopleAction,
                                  wrapped_container = main_sm,
                                  succeeded_outcomes = [],
                                  aborted_outcomes   = [],
                                  preempted_outcomes = ['preempted'] )
    
        # Run the server
        rospy.loginfo("Starting meet_people ActionServerWrapper ...")
        asw.run_server() 
        rospy.loginfo("meet_people ActionServerWrapper started")
    	rospy.spin()
        
    else:
        main_sm.execute()
    
    #rospy.spin()
    sis.stop()


if __name__ == '__main__':
    main()

