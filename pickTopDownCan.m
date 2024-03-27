function can_position = pickTopDownCan()
% Inputs (none)
% Outputs: an array with the [x,y,z,r,p,y] of your end-effector at the 
% time it touches the top of the can

%% Setting up connection to ROS

% rosshutdown
% host_name = '192.168.56.128';
% rosinit(host_name);

%% Call moveTopDownCan
moveTopDownCan;
pause(5);

%% Close end-effector around can 

grip_client = rosactionclient('/gripper_controller/follow_joint_trajectory',...
                              'control_msgs/FollowJointTrajectory');
gripGoal = rosmessage(grip_client);
gripPos = 0.2215;   
gripGoal = packGripGoal(gripPos,gripGoal);
sendGoal(grip_client,gripGoal);

if waitForServer(grip_client)
        disp('Connected to action server. Sending goal...')
        [resultMsg,state,status] = sendGoalAndWait(grip_client, gripGoal);
else
        resultMsg = -1; state = 'failed'; status = 'could not find server';
end

can_position = [0.799, 0.032, 0.122, 3.141, 0.000, 0.000];

end
