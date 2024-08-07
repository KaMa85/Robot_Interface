clear
clc
close all
%%
databaseip="http://localhost";
url = databaseip+'/selectnewest.php';
    apiresponse = webread(url);
    jsonresponse=jsondecode(apiresponse);
if jsonresponse.status=="true"
    currentvideoid = str2num(jsonresponse.id);
end
ind=1;
previouslyknownid=currentvideoid;
while true
    %%%%%connection to the data base Ali,
    while previouslyknownid<currentvideoid
        close all;
        previouslyknownid=currentvideoid;
        url = databaseip+'/selectnewest.php';
        apiresponse = webread(url);
        jsonresponse=jsondecode(apiresponse);
        if jsonresponse.status=="true"
            if rem(ind,2)==1
                x0=str2num(jsonresponse.z_origin)-0.35; y0=-str2num(jsonresponse.x_origin)-0.44; z0=str2num(jsonresponse.y_origin)-0.13;
                x1=str2num(jsonresponse.z_EE)-0.35; y1=-str2num(jsonresponse.x_EE)-0.44;z1=str2num(jsonresponse.y_EE)-0.13; 
                rxh=(str2num(jsonresponse.xrot_EE)); ryh=(str2num(jsonresponse.yrot_EE));rzh=(str2num(jsonresponse.zrot_EE));
            else
                x1=str2num(jsonresponse.z_origin)-0.35; y1=-str2num(jsonresponse.x_origin)-0.44; z1=str2num(jsonresponse.y_origin)-0.13;
                x0=str2num(jsonresponse.z_EE)-0.35; y0=-str2num(jsonresponse.x_EE)-0.44;z0=str2num(jsonresponse.y_EE)-0.13;        
                rxh=(str2num(jsonresponse.xrot_origin)); ryh=(str2num(jsonresponse.yrot_origin));rzh=(str2num(jsonresponse.zrot_origin));
            end
            startpoint = [x0, y0, z0]; 
            endpoint = [x1,y1,z1]; 
            fprintf('start:   %0.3f  ,   %0.3f  ,  %0.3f \n  end:   %0.3f   ,    %0.3f    ,    %0.3f  \n', startpoint,endpoint);
        else
            disp('Not Connected to the Database');
        end
        pause(0.1);

        if 240<rxh && rxh<360 
            ry=rxh-270;
        elseif 0<=rxh && rxh<120
            ry=rxh+90;
        end
        if 240<ryh && ryh<360 
            rz=ryh-270;
        elseif 0<=ryh && ryh<120
            rz=ryh+90;
        end
        if 240<rzh && rzh<360 
            rx=rzh-270;
        elseif 0<=rzh && rzh<120
            rx=rzh+90;
        end
  %% Define and Show Robot at Initial State (Home)
        gen3 = loadrobot("kinovaGen3");%Type 'rigidBodyTree'
        gen3.DataFormat = 'column'; 
        gen3.DataFormat = 'column';
        % q_home_original=[0 60 180 -90 360 -30 180] % q_home2 = [0 15 0 130 0 -55 90] % CARC_Home=[0 340 180 214 0 310 90];   
        q_home=[-90 0 0 0 rz ry rx];
        eeName = 'EndEffector_Link'; %Type 'char'
        T_home = getTransform(gen3, pi*q_home'/180, eeName);
        figure (1)
        show(gen3,pi/180*q_home'); hold on;
        %show(gen3,q_home2);
        axis auto;
        view([45,0]);
        %% Define Path and Direction of Movement
        %Mind the relation between trajectory and time for there are vel limit
        dt = 0.25;
        t = (0:dt:15)';
        theta = t*pi/(2*t(end)); 
        points = startpoint +[(endpoint(1)-startpoint(1))*sin(theta), (endpoint(2)-startpoint(2))*sin(theta), (endpoint(3)-startpoint(3))*sin(theta)];
        hold on;
        plot3(points(:,1),points(:,2),points(:,3),'-*g', 'LineWidth', 1.5);    hold on;
        plot3(startpoint(1),startpoint(2),startpoint(3),'-*r', 'LineWidth', 2.5)
        plot3(endpoint(1),endpoint(2),endpoint(3),'-*b', 'LineWidth', 2.5)
        xlabel('x');
        ylabel('y');
        zlabel('z');
        axis auto;
        % view([60,10]);
        grid('minor');    
    %% Compute Joint Angle Based on Inverse Kinematics of the Path and Animate
        ik = inverseKinematics('RigidBodyTree', gen3);
        ik.SolverParameters.AllowRandomRestart = false;
        weights = [1, 1, 1, 1, 1, 1];
        q_init = pi/180*q_home';
        numJoints = size(pi/180*q_home',1);   
        numWaypoints = size(points,1);
        qs = zeros(numWaypoints,numJoints);
        for i = 1:numWaypoints
            T_des = T_home';
            T_home = getTransform(gen3, pi/180*q_home', eeName);
            T_des(1:3,4) = points(i,:)'; %Desired position, unknown why only row 1-3 of column 4 is changed
            [q_sol, q_info] = ik(eeName, T_des, weights, q_init);
            %Solution information related to execution of the algorithm, q_info, is returned with the joint configuration solution, q_sol.
            % Store the configuration
            qs(i,:) = q_sol(1:numJoints); 
            % Start from prior solution
            q_init = q_sol;
        end
       
        % Animate
        figure (3)
        framesPerSecond = 30;
        r = robotics.Rate(framesPerSecond);
        for i = 1:numWaypoints
            show(gen3, qs(i,:)','PreservePlot',false);
            hold on;
            plot3(points(:,1),points(:,2),points(:,3),'-g','LineWidth',2);
            axis ([-0.5 0.6 -0.4 0.4 0 1]);
            view([45,0]);
            grid('minor');
            drawnow;
            waitfor(r);
        end
        %% Do You Run Kinova?
        prompt = 'Do you want to send same trajectory to the hardware? y/n [n]: ';
        str = input(prompt,'s');
        if isempty(str)
            str = 'n';
        end
        disp([-90 0 0 0 rz ry rx]);

        if str == 'n'
            disp('Operation halted')
            close all;
            break;
        end
        %% Running Kinova
        %Calculate joint velocity and acceleration at each waypoint with differentiation
        qs_deg = qs*180/pi;    
        vel = diff(qs_deg)/dt;
        vel(1,:) = 0;
        vel(end+1,:) = 0; %#ok<SAGROW> 
        acc = diff(vel)/dt;
        acc(1,:) = 0;
        acc(end+1,:) = 0; %#ok<SAGROW> 
        %Interpolate the joint position, velocity and acceleration to ensure the 0.001 seconds time step between two trajectory points
        timestamp = 0:0.001:t(end);
        qs_deg = interp1(t,qs_deg,timestamp);
        vel = interp1(t,vel,timestamp);
        acc = interp1(t,acc,timestamp);
        %Connect to the robot
    %%
        Simulink.importExternalCTypes(which('kortex_wrapper_data.h'));
        gen3Kinova = kortex;
        %gen3Kinova.ip_address = '172.31.99.236';
        gen3Kinova.ip_address = '192.168.1.10'; %Manual ip address
        isOk = gen3Kinova.CreateRobotApisWrapper();
        if isOk
           disp('You are connected to the robot!'); 
        else
           error('Failed to establish a valid connection!'); 
        end
        %%
        f = figure;
        pause(0.1)    
        %Send Robot to Starting Point of the Trajectory
        jointCmd = wrapTo360(qs_deg(1,:));
        constraintType = int32(0); % no_constraint: 0; duration: 1; joint_velocity: 2;
        speed = 0;      %max 25 degrees/s
        duration = 200;   %min 8 seconds
        isOk = gen3Kinova.SendJointAngles(jointCmd, constraintType, speed, duration);
        if isOk
            disp('success');
        else
            disp('SendJointAngles cmd error');
            return;
        end
        pause(0.1)
        %Check if the robot has reached the starting position
        while 1
            [isOk,~, actuatorFb, ~] = gen3Kinova.SendRefreshFeedback();
            if isOk
                if max(abs(wrapTo360(qs_deg(1,:))-actuatorFb.position)) < 1
                    disp('Starting point reached.')
                    break;
                end 
            else
                error('SendRefreshFeedback error')
            end
        end
       
        %Send Pre-Computed Trajectory
        isOk = gen3Kinova.SendPreComputedTrajectory(qs_deg.', vel.', acc.', timestamp, size(timestamp,2));
        if isOk
            disp('SendPreComputedTrajectory success');
        else
            disp('SendPreComputedTrajectory command error');
        end    
        pause(0.1)
        while 1
            [isOk,~, actuatorFb, ~] = gen3Kinova.SendRefreshFeedback();
            if isOk
                if max(abs(wrapTo360(qs_deg(end,:))-actuatorFb.position)) < 1
                    disp('End Point reached.')
                    ind=ind+1;
                    break;
                end 
            else
                error('SendRefreshFeedback error')
            end
        end
    end
    url = databaseip+'/selectnewest.php';
    apiresponse = webread(url);
    jsonresponse=jsondecode(apiresponse);
    if jsonresponse.status=="true"
        currentvideoid = str2num(jsonresponse.id); %#ok<*ST2NM> 
    end
end
%% Retract the robot to origin position
come_home;
%% Retract the robot to origin position
go_initial3;
%% Close gripper in full speed
close_gripper;
%% Open gripper again in full speed
open_gripper;
%% Open gripper again in full speed
go_zero;
%% Test
% This is a good position to turn the robot off so it is balanced
Simulink.importExternalCTypes(which('kortex_wrapper_data.h'));
gen3Kinova = kortex();
%gen3Kinova.ip_address = '172.31.99.236';
gen3Kinova.ip_address = '192.168.1.10'; %Manual ip address
gen3Kinova.user = 'admin';
gen3Kinova.password = 'admin';
isOk = gen3Kinova.CreateRobotApisWrapper();
if isOk
   disp('You are connected to the robot!'); 
else
   error('Failed to establish a valid connection!'); 
end
jointCmd = [-90 0 0 0 91 83 90.7];
constraintType = int32(0);
speed = 0;
duration = 0;
 
isOk = gen3Kinova.SendJointAngles(jointCmd, constraintType, speed, duration);
if isOk
    disp('Command sent to the robot. Wait for robot to finish motion and stop');
else
    error('Command Error.');
end
