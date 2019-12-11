#!/usr/bin/env python

import rospy
import smach
import smach_ros
from smach import StateMachine
import time

#import action headers
import actionlib
#import wrs_fsm.msg
#from wrs_fsm.msg import *
import mobile_path_planning.msg
from mobile_path_planning.msg import *



#enum
from enum import Enum

# Global variables====================
# Shelf selecting
shelf_order = [2,0,3,1,4,5]
shelf_side = 0
shelf_floor = 0
shelf_cnt = 0
motion_cmd = 0

found_none_cnt=0

count = 0
search_cnt = 0

#action result flag
navi_result_flag = False
fsm_motion_result_flag = False

#motion_client Result
found = False
product_id = 0
pose_x = 0.0
pose_y = 0.0
pose_z = 0.0
ori_x = 0.0
ori_y = 0.0
ori_z = 0.0
ori_w = 1.0


#navi_module result
use_marker = 1 #true #static for python?
navi_pose_x = 0.0
navi_pose_y = 0.0
navi_pose_z = 0.0
navi_ori_x = 0.0
navi_ori_y = 0.0
navi_ori_z = 0.0
navi_ori_w = 1.0

# product_count will be set at CHECK_STOCK state
product_cnt = [0, 0, 0, 0, 0, 0]


# ========================================


# ================================================ Result Callback ================================================ #
def navi_result_callback(data):
    global navi_result_flag

    global navi_pose_x, navi_pose_y, navi_pose_z
    global navi_ori_x, navi_ori_y, navi_ori_z, navi_ori_w

    rospy.loginfo("navi Callback result flag: %i", data.result.result_flag)
    navi_result_flag = True

    print data.result.pose_x
    #data update
    navi_pose_x = data.result.pose_x
    navi_pose_y = data.result.pose_y
    navi_pose_z = data.result.pose_z
    navi_ori_x = data.result.ori_x
    navi_ori_y = data.result.ori_y
    navi_ori_z = data.result.ori_z
    navi_ori_w = data.result.ori_w

def fsm_motion_result_callback(data):
    global fsm_motion_result_flag
    global found,product_id, pose_x,pose_y,pose_z,ori_x,ori_y,ori_z,ori_w

    rospy.loginfo("fsm_motion Callback result flag: %i", data.result.result_flag)
    fsm_motion_result_flag = True
    #data update
    found = data.result.found
    product_id = data.result.product_id
    pose_x = data.result.pose_x
    pose_y = data.result.pose_y
    pose_z = data.result.pose_z
    ori_x = data.result.ori_x
    ori_y = data.result.ori_y
    ori_z = data.result.ori_z
    ori_w = data.result.ori_w



#example
#time.sleep(1)

# ================================================ state machine states ================================================ #

# define state IDLE
class IDLE(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome1', 'outcome2'])

    def execute(self, userdata):
        print(" === Starting Store Application Motion ===")
        time.sleep(1)

        if (1):
            return 'outcome1'   # Next: GOTO_DISPLAY
        else:
            return 'outcome2'   # RETURNED_SUCCESS



# ========= Approach to a shelf ========== #

# define state SELECT_SHELF
class SELECT_SHELF(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome1', 'outcome2'])

    def execute(self, userdata):
        print(" === SELECT_SHELF ===")
        time.sleep(1)

        global shelf_cnt

        shelf_side = shelf_order[shelf_cnt]%2 #shelf_side =0:left =1:right
        shelf_floor = shelf_order[shelf_cnt]/2
        shelf_cnt += 1

        if (shelf_cnt < 6):
            return 'outcome1'   # Next: SHELF_OUT
        else:
            return 'outcome2'   # ABORTED

# define state GOTO_DISPLAY
class GOTO_DISPLAY(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome1', 'outcome2'])
        rospy.Subscriber("/navi_dummy/result", naviActionResult, navi_result_callback, queue_size=1, buff_size=10)

    def execute(self, userdata):
        print(" === GOTO_DISPLAY ===")

        global navi_result_flag
        rate = rospy.Rate(10) # 10hz

        global ac_navi
        global navi_pose_x, navi_pose_y, navi_pose_z
        global navi_ori_x, navi_ori_y, navi_ori_z, navi_ori_w

        #Set goal position
        navi_pose_x = 0.0
        navi_pose_y = 1.7
        navi_pose_z = 0.0
        navi_ori_x  = 0.0
        navi_ori_y  = 0.0
        navi_ori_z  = 0.0
        navi_ori_w  = 1.0

        goal_navi = mobile_path_planning.msg.naviGoal(use_marker= use_marker, pose_x=navi_pose_x, pose_y=navi_pose_y, pose_z = navi_pose_z,  ori_w = navi_ori_w, ori_x = navi_ori_x, ori_y = navi_ori_y, ori_z = navi_ori_z )
        ac_navi.send_goal(goal_navi)
        navi_result_flag = False

        # wait in loop until received done flags
        while(navi_result_flag is False):
            rate.sleep()

        if(1):
            return 'outcome1'   # Next: SHELF_OUT
        else:
            return 'outcome2'   # ABORTED

# define state SHELF_OUT
class SHELF_OUT(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome1', 'outcome2'])

    def execute(self, userdata):
        print(" === SHELF_OUT ===")
        time.sleep(5)

        global motion_cmd
        global fsm_motion_result_flag
        rate = rospy.Rate(10) # 10hz

#        global ac_fsm_motion
#        motion_cmd = 1 # enum?
#        goal_fsm_motion = wrs_fsm.msg.fsm_motionGoal(motion_cmd = motion_cmd, shelf_side = shelf_side, shelf_floor = shelf_floor)
#        ac_fsm_motion.send_goal(goal_fsm_motion)
#        fsm_motion_result_flag = False


        # wait in loop until received done flags
#        while(fsm_motion_result_flag is False):
#            rate.sleep()

        if (1):
            return 'outcome1'   # Next: SEARCH_SHELF
        else:
            return 'outcome2'   # ABORTED


# ========= RETURN HOME ========== #
# define state GOTO_HOME
class GOTO_HOME(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome1', 'outcome2'])

    def execute(self, userdata):
        print(" === GOTO_HOME ===")

        global navi_result_flag
        rate = rospy.Rate(10) # 10hz

        global ac_navi
        global navi_pose_x, navi_pose_y, navi_pose_z
        global navi_ori_x, navi_ori_y, navi_ori_z, navi_ori_w

        navi_pose_x = 0.0
        navi_pose_y = -0.2
        navi_pose_z = 0.0
        navi_ori_x  = 0.0
        navi_ori_y  = 0.0
        navi_ori_z  = 0.0
        navi_ori_w  = 1.0

        goal_navi = mobile_path_planning.msg.naviGoal(use_marker= use_marker, pose_x=navi_pose_x, pose_y=navi_pose_y, pose_z = navi_pose_z, ori_w = navi_ori_w, ori_x = navi_ori_x, ori_y = navi_ori_y, ori_z = navi_ori_z )
        ac_navi.send_goal(goal_navi)
        navi_result_flag = False

        # wait in loop until received done flags
        while(navi_result_flag is False):
            rate.sleep()


        if (1):
            return 'outcome1'   # Next: SELECT_SHELF
        else:
            return 'outcome2'   # ABORTED

# ================================================ main loop ================================================ #


def main():
    #initialize node
    rospy.init_node('navi_module')


    #action variables
    global ac_navi, ac_fsm_motion

    # Creates the SimpleActionClient, passing the type of the action to the constructor.
    ac_navi = actionlib.SimpleActionClient('hubo_navigation', mobile_path_planning.msg.naviAction)

    # Waits until the action server has started up and started
    # listening for goals.
    ac_navi.wait_for_server()

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['ABORTED', 'RETURNED_SUCCESS', 'RETURNED_WITHOUT'])

    # Open the container
    with sm:

        # Add states to the container
        smach.StateMachine.add('IDLE', IDLE(),
                               transitions={'outcome1': 'SELECT_SHELF', 'outcome2':'RETURNED_SUCCESS'})

        # === Select a shelf
        smach.StateMachine.add('SELECT_SHELF', SELECT_SHELF(),
                               transitions={'outcome1': 'GOTO_DISPLAY', 'outcome2':'RETURNED_SUCCESS'})

        smach.StateMachine.add('GOTO_DISPLAY', GOTO_DISPLAY(),
                               transitions={'outcome1': 'SHELF_OUT', 'outcome2':'RETURNED_SUCCESS'})

        smach.StateMachine.add('SHELF_OUT', SHELF_OUT(),
                                transitions={'outcome1': 'GOTO_HOME', 'outcome2':'ABORTED'})

        # === GOTO_HOME
        smach.StateMachine.add('GOTO_HOME', GOTO_HOME(),
                                transitions={'outcome1': 'RETURNED_SUCCESS', 'outcome2':'ABORTED'})


    # Create and start the introspection server (for smach viewer)
    sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT/naviModule_test')
    sis.start()


    # Execute SMACH plan
    outcome = sm.execute()

    # Wait for ctrl-c to stop the application
    rospy.spin()
    sis.stop()

if __name__ == '__main__':
    main()
