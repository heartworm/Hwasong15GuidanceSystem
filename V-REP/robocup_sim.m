% Make sure to have the server side running in V-REP:
% in a child script of a V-REP scene, add following command
% to be executed just once, at simulation start:
%
% simExtRemoteApiStart(19999)
%
% then start simulation, and run this program.
%
% IMPORTANT: for each successful call to simxStart, there
% should be a corresponding call to simxFinish at the end!
clear all

	disp('Program started');
	% vrep=remApi('remoteApi','extApi.h'); % using the header (requires a compiler)
	vrep=remApi('remoteApi'); % using the prototype file (remoteApiProto.m)
	vrep.simxFinish(-1); % just in case, close all opened connections
	clientID=vrep.simxStart('127.0.0.1',19999,true,true,5000,5);

	if (clientID>-1)
		disp('Connected to remote API server');

		% Now try to retrieve data in a blocking fashion (i.e. a service call):
		[res,objs]=vrep.simxGetObjects(clientID,vrep.sim_handle_all,vrep.simx_opmode_blocking);
		if (res==vrep.simx_return_ok)
			fprintf('Number of objects in the scene: %d\n',length(objs));
		else
			fprintf('Remote API function call returned with error code: %d\n',res);
        end

        % Now send some data to V-REP in a non-blocking fashion:
		vrep.simxAddStatusbarMessage(clientID,'Matlab Connected to Robocup Simulator!', vrep.simx_opmode_oneshot);

		pause(2);

		% Now retrieve streaming data (i.e. in a non-blocking fashion):
		t=clock;
		startTime=t(6);
		currentTime=t(6);
        wheel_diameter = 0.085;

        [~, cameraHandle] = vrep.simxGetObjectHandle(clientID,'vision_sensor',vrep.simx_opmode_blocking);
        [~, leftWheelHandle] = vrep.simxGetObjectHandle(clientID,'left_wheel_joint',vrep.simx_opmode_blocking);
        [~, rightWheelHandle] = vrep.simxGetObjectHandle(clientID,'right_wheel_joint',vrep.simx_opmode_blocking);
        [~, robotHandle] = vrep.simxGetObjectHandle(clientID,'robot',vrep.simx_opmode_blocking);
        [~, ballHandle] = vrep.simxGetObjectHandle(clientID,'ball',vrep.simx_opmode_blocking);
        [~, obstacleHandle] = vrep.simxGetObjectHandle(clientID,'obstacle',vrep.simx_opmode_blocking);

		vrep.simxGetVisionSensorImage2(clientID,cameraHandle,0,vrep.simx_opmode_streaming); % Initialize streaming
		vrep.simxGetObjectPosition(clientID,ballHandle,-1,vrep.simx_opmode_streaming);

		%This will only run for 10 seconds, so make sure to remove this if a longer time is needed
        first_call = true;
        while (currentTime-startTime < 50)
            if(first_call)
                call_mode = vrep.simx_opmode_streaming;
            else
                call_mode = vrep.simx_opmode_buffer;
            end
			%[returnCode,data]=vrep.simxGetIntegerParameter(clientID,vrep.sim_intparam_mouse_x,vrep.simx_opmode_buffer); % Try to retrieve the streamed data
			[returnCode,resolution,image]=vrep.simxGetVisionSensorImage2(clientID,cameraHandle,0,vrep.simx_opmode_buffer);
            if (returnCode==vrep.simx_return_ok) % After initialization of streaming, it will take a few ms before the first value arrives, so check the return code
              	% Do Image processing here
                imshow(image);
                %perception(image);
            end
            
            % use this if not using image processing to find object
            % positions, (note - using "~" to ignore the returnCode
            % variable)

            [~,ballPosition]=vrep.simxGetObjectPosition(clientID,ballHandle,-1,call_mode);
                       %disp(ballPosition);
            [~,obstaclePosition]=vrep.simxGetObjectPosition(clientID,obstacleHandle,-1,call_mode);
                       %disp(obstaclePosition);
            [~,robotPosition]=vrep.simxGetObjectPosition(clientID,robotHandle,-1,call_mode);
                       %disp(robotPosition);
            [~,robotOrientation]=vrep.simxGetObjectOrientation(clientID,robotHandle,-1,call_mode);
                       %disp(robotOrientation);
            [~,robotLinearVelocity, robotAngularVelocity]=vrep.simxGetObjectVelocity(clientID,robotHandle,call_mode);
                       %disp(robotLinearVelocity);
                       %disp(robotAngularVelocity);                     

            %setting goal position as ball position, this could be
            %something else
            goal_position.x = ballPosition(1);
            goal_position.y = ballPosition(2);
            
            obstacle.x = obstaclePosition(1);
            obstacle.y = obstaclePosition(2);
            %can be an array of obstacles = [obstacle1, obstacle2]
                       
            robot.x = robotPosition(1); %x coordinate
            robot.y = robotPosition(2); %y coordinate
            robot.angle = convert_180_to_360_angle(rad2deg(robotOrientation(3))); %global angle around z axis
            robot.x_vel = robotLinearVelocity(1); %x velocity
            robot.y_vel = robotLinearVelocity(2); %y velocity
            robot.rot_vel = robotAngularVelocity(3); %rotational velocity around z axis in radians
            
		    % Do some smart things here (AI + Navigation)
            if(~first_call)
                %this calls a navigation function which you should
                %implement
                %[right_wheel_velocity, left_wheel_velocity, attractive_field, repulsive_field, residual_field] = navigation(goal_position, obstacle, robot, true);
            
                right_wheel_velocity = 0.1
                left_wheel_velocity = 0.1
                
                disp('right_wheel_velocity')
                disp(right_wheel_velocity)
                disp('left_wheel_velocity')
                disp(left_wheel_velocity)
            
                % Apply wheel comman    ds by setting left and right wheel velocities
                vrep.simxSetJointTargetVelocity(clientID,leftWheelHandle,left_wheel_velocity*2/wheel_diameter,vrep.simx_opmode_blocking );
                vrep.simxSetJointTargetVelocity(clientID,rightWheelHandle,right_wheel_velocity*2/wheel_diameter,vrep.simx_opmode_blocking );
            else
                first_call = false;
            end
            t=clock;
			currentTime=t(6);
		end


		% Now close the connection to V-REP:
		vrep.simxFinish(clientID);
	else
		disp('Failed connecting to remote API server');
	end
	vrep.delete(); % call the destructor!

	disp('Program ended');
  
