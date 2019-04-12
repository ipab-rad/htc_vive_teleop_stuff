#!/usr/bin/env python

# import sys
import rospy
from moveit_msgs.srv import GetStateValidityRequest
from moveit_msgs.srv import GetStateValidity
from sensor_msgs.msg import JointState

def get_check_state_service(persistent=True):
    rospy.wait_for_service('/check_state_validity')
    try:
        isStateValidService = rospy.ServiceProxy(
            '/check_state_validity', GetStateValidity, persistent=persistent)
        return isStateValidService
    except rospy.ServiceException, e:
        print("Service call failed: %s"%e)
        return None

def check_state(names, positions, service=None, verbose=False):
    # Fill in robot state in call

    assert len(names) > 0
    assert len(names) == len(positions)

    jntState = JointState()
    jntState.header.stamp = rospy.Time.now()
    jntState.name = names
    jntState.position = list(positions)
    jntState.velocity = [.0] * len(positions)
    jntState.effort = [.0] * len(positions)

    state_validity = GetStateValidityRequest()
    state_validity.robot_state.joint_state = jntState
    # state_validity.group_name = 'right_arm'
    # state_validity.constraints

    try:
        if service is None:
            isStateValidService = get_check_state_service(persistent=False)
        else:
            isStateValidService = service
        resp1 = isStateValidService(state_validity)
        if verbose:
            print('resp: ', resp1, resp1.valid)
        return resp1.valid
    except rospy.ServiceException, e:
        print("Service call failed: %s"%e)
        return False

if __name__ == "__main__":
    import time

    right_joint_names = ["r_shoulder_pan_joint", "r_shoulder_lift_joint", "r_upper_arm_roll_joint",
                      "r_elbow_flex_joint", "r_forearm_roll_joint", "r_wrist_flex_joint", "r_wrist_roll_joint"]
    left_joint_names = ["l_shoulder_pan_joint", "l_shoulder_lift_joint", "l_upper_arm_roll_joint",
                      "l_elbow_flex_joint", "l_forearm_roll_joint", "l_wrist_flex_joint", "l_wrist_roll_joint"]                      

    rospy.init_node('service_state_checker')

    state = [1.5]*7
    start = time.time()
    print(state, check_state(right_joint_names, state, verbose=False))
    print('Time took:', (time.time() - start)*1000, ' ms')
    state = [0.5]*7
    start = time.time()
    print(state, check_state(right_joint_names, state, verbose=False))
    print('Time took:', (time.time() - start)*1000, ' ms')

    service = get_check_state_service(persistent=True)
    state = [1.5]*7
    start = time.time()
    print(state, check_state(right_joint_names, state, service=service, verbose=False))
    print('Time took:', (time.time() - start)*1000, ' ms')
    state = [0.5]*7
    start = time.time()
    print(state, check_state(right_joint_names, state, service=service, verbose=False))
    print('Time took:', (time.time() - start)*1000, ' ms')
    state = [-0.5]*7
    start = time.time()
    print(state, check_state(right_joint_names, state, service=service, verbose=False))
    print('Time took:', (time.time() - start)*1000, ' ms')



    print('Done.')
