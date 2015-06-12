#!/usr/bin/env python
import roslib; roslib.load_manifest('reemc_grasping')
import rospy
import actionlib
import play_motion_msgs.msg as PMM

if __name__ == '__main__':
  rospy.init_node('back_to_home')
  play_motion = actionlib.SimpleActionClient('/play_motion', PMM.PlayMotionAction)
  play_motion.wait_for_server()
  goal = PMM.PlayMotionGoal(motion_name='arms_t', skip_planning=False)
  play_motion.send_goal(goal)
  play_motion.wait_for_result()
  rospy.sleep(2)
  goal = PMM.PlayMotionGoal(motion_name='interact', skip_planning=False)
  play_motion.send_goal(goal)
  play_motion.wait_for_result()
