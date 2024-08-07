clear
clc
close all
%%
% Positional Transformation
ax=0.705;
ay=0.89;
az=-0.033;
currentvideoid=0;
gripper_state=2;
databaseip="http://localhost";
url_from_HL = databaseip+'/selectnewest.php';
url_to_HL = databaseip+'/selectnewest.php';
    apiresponse = webread(url_from_HL);
    jsonresponse=jsondecode(apiresponse);
if jsonresponse.status=="true"
    currentvideoid = str2num(jsonresponse.id); %#ok<*ST2NM>  
end
Simulink.importExternalCTypes(which('kortex_wrapper_data.h'));
gen3Kinova = kortex();
%gen3Kinova.ip_address = '172.31.99.235';
gen3Kinova.ip_address = '192.168.1.10'; %Manual ip address
gen3Kinova.user = 'admin';
gen3Kinova.password = 'admin';
isOk = gen3Kinova.CreateRobotApisWrapper();
if isOk
   disp('You are connected to the robot!'); 
else
   error('Failed to establish a valid connection!'); 
end
ind=1;
previouslyknownid=currentvideoid;
%%
while true
    %%%%connection to the data base Ali,
    while previouslyknownid<currentvideoid
        close all;
        previouslyknownid=currentvideoid;
        url_from_HL = databaseip+'/selectnewest.php';
        apiresponse = webread(url_from_HL);
        jsonresponse=jsondecode(apiresponse);
        if jsonresponse.status=="true"
            gripper_state=str2num(jsonresponse.EE_state);
            if gripper_state==1
                disp(gripper_state);
                gripper_state=2;
                toolCommand = int32(2);  
                toolCmd = 1;
                toolDuration = 0;
                isOk = gen3Kinova.SendToolCommand(toolCommand, toolDuration, toolCmd);
                if isOk
                    disp('Command sent to the gripper. Wait for the gripper to open.')
                else
                    error('Command Error.');
                end
                break;
            elseif gripper_state==0
                disp(gripper_state);
                gripper_state=2;
                toolCommand = int32(2);    % Velocity control mode
                toolDuration = 0;
                toolCmd = -1; % Close the gripper with full speed
                isOk = gen3Kinova.SendToolCommand(toolCommand, toolDuration, toolCmd);
                if isOk
                    disp('Command sent to the gripper. Wait for the gripper to close.')
                else
                    error('Command Error.');
                end
                break;
            end
            if rem(ind,2)==1
                x1=str2num(jsonresponse.z_EE)+ax; y1=-str2num(jsonresponse.x_EE)+ay;z1=str2num(jsonresponse.y_EE)+az;   
                x0=str2num(jsonresponse.z_origin)+ax; y0=-str2num(jsonresponse.x_origin)+ay; z0=str2num(jsonresponse.y_origin)+az;
                rxx=str2num(jsonresponse.rot_go11); rxy=str2num(jsonresponse.rot_go12);rxz=str2num(jsonresponse.rot_go13);
                ryx=str2num(jsonresponse.rot_go21); ryy=str2num(jsonresponse.rot_go22);ryz=str2num(jsonresponse.rot_go23);
                rzx=str2num(jsonresponse.rot_go31); rzy=str2num(jsonresponse.rot_go32);rzz=str2num(jsonresponse.rot_go33);
                rxx0=str2num(jsonresponse.rot_origin11); rxy0=str2num(jsonresponse.rot_origin12);rxz0=str2num(jsonresponse.rot_origin13);
                ryx0=str2num(jsonresponse.rot_origin21); ryy0=str2num(jsonresponse.rot_origin22);ryz0=str2num(jsonresponse.rot_origin23);
                rzx0=str2num(jsonresponse.rot_origin31); rzy0=str2num(jsonresponse.rot_origin32);rzz0=str2num(jsonresponse.rot_origin33);
            else
                 x1=str2num(jsonresponse.z_origin)+ax; y1=-str2num(jsonresponse.x_origin)+ay; z1=str2num(jsonresponse.y_origin)+az;
                 x0=str2num(jsonresponse.z_EE)+ax; y0=-str2num(jsonresponse.x_EE)+ay;z0=str2num(jsonresponse.y_EE)+az;   
                rxx=str2num(jsonresponse.rot_origin11); rxy=str2num(jsonresponse.rot_origin12);rxz=str2num(jsonresponse.rot_origin13);
                ryx=str2num(jsonresponse.rot_origin21); ryy=str2num(jsonresponse.rot_origin22);ryz=str2num(jsonresponse.rot_origin23);
                rzx=str2num(jsonresponse.rot_origin31); rzy=str2num(jsonresponse.rot_origin32);rzz=str2num(jsonresponse.rot_origin33);
                rxx0=str2num(jsonresponse.rot_go11); rxy0=str2num(jsonresponse.rot_go12);rxz0=str2num(jsonresponse.rot_go13);
                ryx0=str2num(jsonresponse.rot_go21); ryy0=str2num(jsonresponse.rot_go22);ryz0=str2num(jsonresponse.rot_go23);
                rzx0=str2num(jsonresponse.rot_go31); rzy0=str2num(jsonresponse.rot_go32);rzz0=str2num(jsonresponse.rot_go33);
            end
            startpoint = [x0, y0, z0]; 
            endpoint = [x1,y1,z1]; 
            fprintf('start:   %0.3f  ,   %0.3f  ,  %0.3f \n  end:   %0.3f   ,    %0.3f    ,    %0.3f  \n', startpoint,endpoint);
        else
            disp('Not Connected to the Database');
        end
        pause(0.1);
  %% Define and Show Robot at Initial State (Home)
        gen3 = loadrobot("kinovaGen3");%Type 'rigidBodyTree'
        gen3.DataFormat = 'column'; 
        eeName = 'EndEffector_Link'; %Type 'char'
        initial_config = gen3.homeConfiguration;
        ik = inverseKinematics('RigidBodyTree', gen3);
        ik.SolverParameters.AllowRandomRestart = false;
        weights0 = [1, 1, 1, 1, 1, 1];
        T_home = getTransform(gen3, pi*initial_config/180, eeName);
        T_des = getTransform(gen3, pi*initial_config/180, eeName);
        t= [0,0,1;1,0,0;0,1,0];
        T_des(1:3,1:3) =t*[rxx, -rxy,-rxz;-ryx,ryy,ryz;-rzx,rzy,rzz];     
        T_home(1:3,1:3) =t*[rxx0, -rxy0,-rxz0;-ryx0,ryy0,ryz0;-rzx0,rzy0,rzz0];     
        [q_home,] = ik(eeName, T_home, weights0, initial_config); q_home=q_home'*180/pi;
        [q_des,] = ik(eeName, T_des, weights0, initial_config); q_des=q_des'*180/pi;
        figure (1)
        show(gen3,pi/180*q_home'); hold on;
        axis auto;
        view([45,0]);
        %% Define Path and Direction of Movement
        %Mind the relation between trajectory and time for there are vel limit
        dt = 0.25;
        dt2 = 0.25;
        t = (0:dt:15)';
        t2 = (0:dt2:15)';
        theta = t*pi/(2*t(end)); 
        theta2 = t2*pi/(2*t2(end)); 
        points = startpoint +[(endpoint(1)-startpoint(1))*sin(theta), (endpoint(2)-startpoint(2))*sin(theta), (endpoint(3)-startpoint(3))*sin(theta)];
        pointsAR = [-y0+ay,z0-az,x0-ax] +[(-y1+y0)*sin(theta), (z1-z0)*sin(theta), (x1-x0)*sin(theta)];
        %upload photo and data to database
        jsonPoints=jsonencode(pointsAR);
        disp(jsonPoints);
        response = webwrite(databaseip+'/insert.php','json_points',jsonPoints);

        q=zeros(length(points),length(q_home)); q(:,1)=q_home(1):(q_des(1)-q_home(1))/(length(points)-1):q_des(1);
        q(:,2)=q_home(2):(q_des(2)-q_home(2))/(length(points)-1):q_des(2);q(:,3)=q_home(3):(q_des(3)-q_home(3))/(length(points)-1):q_des(3);
        q(:,4)=q_home(4):(q_des(4)-q_home(4))/(length(points)-1):q_des(4); q(:,5)=q_home(5):(q_des(5)-q_home(5))/(length(points)-1):q_des(5);
        q(:,6)=q_home(6):(q_des(6)-q_home(6))/(length(points)-1):q_des(6);q(:,7)=q_home(7):(q_des(7)-q_home(7))/(length(points)-1):q_des(7);
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
        q_init = pi/180*q(1,:)';
        numJoints = size(pi/180*q_home',1);   
        numWaypoints = size(points,1);
        qs = zeros(numWaypoints,numJoints);
        for i = 1:numWaypoints
            T_des = T_home;
            T_home = getTransform(gen3, pi/180*q(i,:)', eeName);
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
        % Do You Run Kinova?
        prompt = 'Do you want to send same trajectory to the hardware? y/n [n]: ';
        str = input(prompt,'s');
        %str='y'; %ind=ind+1;
        if isempty(str)
            str = 'n';
        end
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
    url_from_HL = databaseip+'/selectnewest.php';
    apiresponse = webread(url_from_HL);
    jsonresponse=jsondecode(apiresponse);
    if jsonresponse.status=="true"
        currentvideoid = str2num(jsonresponse.id); %#ok<*ST2NM> 
    end
end
%% Retract the robot to origin position
come_home;
%% Retract the robot to origin position
go_initial1;
%% Close gripper in full speed
close_gripper;
%% Open gripper again in full speed
open_gripper;
%% Open gripper again in full speed
go_zero;
%% Test
This is a good position to turn the robot off so it is balanced
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
