function can_position = moveToCan()

% Inputs: none
% Outputs: an array with the [x,y,z,r,p,y] of your closed end-effector 
% at the time it touches the top of the can.

%% Setting up connection to ROS

rosshutdown
host_name = '192.168.56.128';
rosinit(host_name);

%% Getting the locations of objects

model_states_sub = rossubscriber("/gazebo/model_states", "DataFormat", "struct");
model_states_data = receive(model_states_sub);
rCan3_position = model_states_data.Pose(26).Position;

%% Setting up an action to move the robots gripper above can, rCan3

robot_client = rosactionclient('/pos_joint_traj_controller/follow_joint_trajectory',...
                              'control_msgs/FollowJointTrajectory');
robotGoal = rosmessage(robot_client); % sets up the end goal for the robot position
jointSub = rossubscriber("/joint_states"); % listens to '/joint_states' topic for current position

UR5e = loadrobot('universalUR5e', DataFormat="row"); % setup UR5e robot model

tform=UR5e.Bodies{3}.Joint.JointToParentTransform;    
UR5e.Bodies{3}.Joint.setFixedTransform(tform*eul2tform([pi/2,0,0]));

tform=UR5e.Bodies{4}.Joint.JointToParentTransform;
UR5e.Bodies{4}.Joint.setFixedTransform(tform*eul2tform([-pi/2,0,0]));

tform=UR5e.Bodies{7}.Joint.JointToParentTransform;
UR5e.Bodies{7}.Joint.setFixedTransform(tform*eul2tform([-pi/2,0,0]));

ik = inverseKinematics("RigidBodyTree",UR5e); % Create Inverse kinematics solver
ikWeights = [0.25 0.25 0.25 0.1 0.1 .1]; % configuration weights for IK solver [Translation Orientation] see documentation

jointStateCurrent = receive(jointSub);
initialIKGuess = homeConfiguration(UR5e);

initialIKGuess(1) = jointStateCurrent.Position(4); % update configuration in initial guess
initialIKGuess(2) = jointStateCurrent.Position(3);
initialIKGuess(3) = jointStateCurrent.Position(1);
initialIKGuess(4) = jointStateCurrent.Position(5);
initialIKGuess(5) = jointStateCurrent.Position(6);
initialIKGuess(6) = jointStateCurrent.Position(7);

gripperX = rCan3_position.X;
gripperY = rCan3_position.Y;
gripperZ = rCan3_position.Z;

gripperTranslation = [(-gripperY) (gripperX + 0.1) (gripperZ - 0.45)];%0.005
gripperRotation = [-pi/2 -pi 0]; %  [Z Y Z] radians


tform = eul2tform(gripperRotation); % ie eul2tr call
tform(1:3,4) = gripperTranslation'; % set translation in homogeneous transform

[configSoln, solnInfo] = ik('tool0',tform,ikWeights,initialIKGuess);

UR5econfig = [configSoln(3)... 
              configSoln(2)...
              configSoln(1)...
              configSoln(4)...
              configSoln(5)...
              configSoln(6)];

robotGoal = packTrajGoal(UR5econfig,robotGoal);
sendGoal(robot_client,robotGoal);

can_position = [gripperTranslation(1), gripperTranslation(2), gripperTranslation(3),...
                gripperRotation(1), gripperRotation(2), gripperRotation(3)];
end