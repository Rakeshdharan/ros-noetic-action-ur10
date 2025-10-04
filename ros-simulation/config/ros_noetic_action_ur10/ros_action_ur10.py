import os
import sys
sys.path.append("/opt/ros/noetic/lib/python3/dist-packages")
import scipy
import numpy as np
import roboticstoolbox as rtb
from spatialmath import SE3
from spatialmath.base import *
import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
import time
import matplotlib.pyplot as plt
from helperFunctions import JointVec2JointTrajectoryMsg, JointVec2FollowJointTrajectoryMsg
import actionlib
import control_msgs.msg

name_list = ["shoulder_pan_joint","shoulder_lift_joint","elbow_joint","wrist_1_joint","wrist_2_joint","wrist_3_joint"]
jointStateMsg = JointState()

def jointStateCallback(msg):
    global jointStateMsg
    jointStateMsg = msg

# import ur10.urdf
notebook_path = os.getcwd()
ur10_file_path = notebook_path + "/ur10.urdf"
ur10 = rtb.Robot.URDF(ur10_file_path)

# initialize an inverse kinematic solver
mask = [1, 1, 1, 1, 1, 1]
ik = rtb.IK_LM(mask=mask)

targetPosition = [0.6, 0, 1.0]
targetOrientation = [-np.pi / 4, np.pi / 4, np.pi / 4]

tformTrans = SE3(targetPosition)
tformRot = SE3(rpy2tr(targetOrientation))
tformTargetPose = tformTrans @ tformRot

initial_guess = [0, 1, -2, 2, 1, 1]
targetConf, _, _, _, _, _ = ik.solve(ur10.ets(), tformTargetPose, initial_guess)

rospy.init_node("ros_motion_ur10")

rospy.Subscriber('/ur10/joint_states', JointState, jointStateCallback)

# create publisher
jointTrajectoryPub = rospy.Publisher(
    "/ur10/vel_based_pos_traj_controller/command", JointTrajectory, queue_size=10
)

# convert target configuration
q_target = targetConf
t_target = 2.5

# time offset to let robot finish motion
t_offset = 3  # increase if robot does not reach it's configuration in time

# send target configuration to the robot
jointTrajectoryMsg = JointVec2JointTrajectoryMsg(q_target, t_target)
jointTrajectoryMsg.header.stamp = rospy.Time.now()
jointTrajectoryPub.publish(jointTrajectoryMsg)

time.sleep(t_target + t_offset)

# Move to start configuration
q_home = np.array([0, 0, 0, 0, 0, 0])
t_home = 2.5

print("EE moves to home config or already at home config")
jointTrajectoryMsg = JointVec2JointTrajectoryMsg(q_home, t_home)
jointTrajectoryMsg.header.stamp = rospy.Time.now()
jointTrajectoryPub.publish(jointTrajectoryMsg)

time.sleep(t_home + t_offset)
# Define target joint position and joint velocity
qvel = np.array([0, 0, 0, 0, 0, 0])

# Create rate object
rate = 50
# Hz
# rateObj=robotics.Rate(rate);
rateObj = rospy.Rate(rate)
tf = t_target + 0.5
# rateObj.reset; # reset time of rate object

# Preallocation
N = int(tf * rate)
timeStamp = np.zeros((N, 1))
jointStateStamped = np.zeros((N, 6))  # array to record joint pose
jointVelStamped = np.zeros((N, 6))  # array to record joint velocity

# Store start time
t0 = jointStateMsg.header.stamp.to_sec()

# Move and monitor
print("EE moves to Final/Target Pose from home config")
jointTrajectoryMsg = JointVec2JointTrajectoryMsg(q_target, t_target)
jointTrajectoryMsg.header.stamp = rospy.Time.now()
jointTrajectoryPub.publish(jointTrajectoryMsg)

for i in range(1, N):
    # Receive and convert
    jointState = jointStateMsg.position
    jointVel = jointStateMsg.velocity
    # Store signals
    jointStateStamped[i, :] = jointState
    jointVelStamped[i, :] = jointVel
    timeStamp[i] = jointStateMsg.header.stamp.to_sec() - t0  # + msg_time*10^-9

    rateObj.sleep()

# Back to home
print("\nGoing back to home config")
jointTrajectoryMsg = JointVec2JointTrajectoryMsg(q_home, t_home)
jointTrajectoryMsg.header.stamp = rospy.Time.now()
jointTrajectoryPub.publish(jointTrajectoryMsg)

time.sleep(t_home + t_offset)

# Create second waypoint
t_targets = [2.5, 6.0]
qf = np.array([-1, 0, -1, 1, 0, 2])
q_targets = np.array([q_target, qf])

#Intermediate waypoint velocity
qvel1 = np.array([-0.4, 0, 0, -0.3, 0, 0])

#Final waypoint velocity
qvelf = np.zeros(6)
qvel = np.array([qvel1, qvelf])

tf = t_targets[-1] + 1.0
# rateObj.reset; # reset time of rate object

# Preallocation
N = int(tf * rate)
timeStamp = np.zeros((N, 1))
jointStateStamped = np.zeros((N, 6))  # array to record joint pose
jointVelStamped = np.zeros((N, 6))  # array to record joint velocity

# Store start time
t0 = jointStateMsg.header.stamp.to_sec()


#define the action client with the respective topic
followJointTrajectoryTopicName = '/ur10/vel_based_pos_traj_controller/follow_joint_trajectory'
followJointTrajectoryActClient = actionlib.SimpleActionClient(
    followJointTrajectoryTopicName, control_msgs.msg.FollowJointTrajectoryAction
)
followJointTrajectoryActClient.wait_for_server()
followJointTrajectoryMsg = control_msgs.msg.FollowJointTrajectoryGoal()


# Back to home
'''jointTrajectoryMsg = JointVec2JointTrajectoryMsg(q_home, t_home)
jointTrajectoryMsg.header.stamp = rospy.Time.now()
jointTrajectoryPub.publish(jointTrajectoryMsg)'''

time.sleep(t_home + t_offset)

# Create action msg
followJointTrajectoryMsg = JointVec2FollowJointTrajectoryMsg(q_targets, t_targets, qvel);


# Send message
exec_timeout = rospy.Duration(10)
prmpt_timeout = rospy.Duration(5)

print("Send goal and wait (Blocking method), UR10 EE moves to the intermediate waypoint and then continues to target config")
resultState = followJointTrajectoryActClient.send_goal_and_wait(followJointTrajectoryMsg, exec_timeout, prmpt_timeout)
resultMsg = followJointTrajectoryActClient.get_result()

# Evaluate result
if (resultMsg.error_code):
    print('Arm motion error')
    print(resultMsg)
else:
    print('UR arm motion completed with state ' + str(resultState) + '.')


# Back to home
print("\nGoing back to home config and preparing for next experiment")
jointTrajectoryMsg = JointVec2JointTrajectoryMsg(q_home,t_home)
jointTrajectoryMsg.header.stamp = rospy.Time.now()
jointTrajectoryPub.publish(jointTrajectoryMsg)

time.sleep(t_home + t_offset)

# Create action msg
followJointTrajectoryMsg = JointVec2FollowJointTrajectoryMsg(q_targets, t_targets, qvel);

# Send message
exec_timeout = rospy.Duration(10)
prmpt_timeout = rospy.Duration(5)

finished = False

#callback to read the status of the non-blocking method
def done_cb(status, result):
    global finished, resultMsg, resultState
    if status == 3:
        finished = True
        resultState = status
    else:
        pass
    resultMsg = result

timeStamp=[]
jointStateStamped=[]
jointVelStamped=[]
t0 = jointStateMsg.header.stamp.to_sec()
print("Send goal and wait (Non-blocking method), fire and forget method")
resultState = followJointTrajectoryActClient.send_goal(followJointTrajectoryMsg, done_cb)
i = 0

while not finished:

    # Receive and convert
    #jointStateMsg=jointStateSub.receive()
    #[jointState, jointVel]=JointStateMsg2JointState(ur10,jointStateMsg);
    jointState = jointStateMsg.position
    jointVel = jointStateMsg.velocity
    
    # Store signals
    jointStateStamped.append(jointState)
    jointVelStamped.append(jointVel)
    timeStamp.append(jointStateMsg.header.stamp.to_sec() - t0)# + msg_time*10^-9
    rateObj.sleep()
    i+=1
jointStateStamped = np.stack(jointStateStamped, axis=0)
jointVelStamped = np.stack(jointVelStamped, axis=0)
timeStamp = np.stack(timeStamp, axis=0)
# Evaluate result
if (resultMsg.error_code):
    print('Arm motion error')
    print(resultMsg)
else:
    print('UR arm motion completed with state ' + str(resultState) + '.')



