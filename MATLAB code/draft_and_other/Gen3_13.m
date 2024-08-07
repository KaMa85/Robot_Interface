clear
clc
close all
%%
% Positional Transformation
ax=0.698;
ay=0.915;
az=-0.0;
currentvideoid=0;
gripper_state=2;
databaseip="http://localhost";
url_from_HL = databaseip+'/selectnewest.php';
url_to_HL = databaseip+'/insert.php';
decision = databaseip+'/selectdecision.php';
apiresponse = webread(url_from_HL);
jsonresponse=jsondecode(apiresponse);
if jsonresponse.status=="true"
    currentvideoid = str2num(jsonresponse.id); %#ok<*ST2NM>  
end
previouslyknownid=currentvideoid;
apidecision = webread(decision);
jsondecision=jsondecode(apidecision);
if jsondecision.status=="true"
    currentid = str2num(jsonresponse.id); %#ok<*ST2NM>  
end
previousid=currentid;
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
%%
while true
    %%%%connection to the data base Ali,
    while previouslyknownid<currentvideoid
        close all;
        previouslyknownid=currentvideoid;
        apiresponse =  webread(url_from_HL);
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
            currentvideoid = str2num(jsonresponse.id); %#ok<*ST2NM>
            lrx=str2num(jsonresponse.lr_z)+ax;lry=-str2num(jsonresponse.lr_x)+ay;lrz=str2num(jsonresponse.lr_y)+az;
            llx=str2num(jsonresponse.ll_z)+ax;lly=-str2num(jsonresponse.ll_x)+ay;llz=str2num(jsonresponse.ll_y)+az;
            urx=str2num(jsonresponse.ur_z)+ax;ury=-str2num(jsonresponse.ur_x)+ay;urz=str2num(jsonresponse.ur_y)+az;
            ulx=str2num(jsonresponse.ul_z)+ax;uly=-str2num(jsonresponse.ul_x)+ay;ulz=str2num(jsonresponse.ul_y)+az;
            lr = [lrx, lry,lrz];ll = [llx, lly,llz];ur = [urx, ury,urz];ul = [ulx, uly,ulz];
            constr=[lr;ur;ul;ll];
            writematrix(constr,'constr','Delimiter','tab')
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
        %% path planning
        % initialize rrt
        rrt = RrtPlanner(25,3,1);
        rrt.SetStart(startpoint);
        rrt.SetGoal(endpoint);
        rrt.SetUpDataStructures();
        rrt.Run()
        delete(rrt);
        % obstacle
        obstacleFilename = 'constr.txt';
        % final rrt
        seedsPerAxis = 7;
        treesMax = seedsPerAxis^3*3+2;
        rrt = RrtPlanner(treesMax,seedsPerAxis,obstacleFilename);
        rrt.drawingSkipsPerDrawing = 30;
        rrt = RrtPlanner(treesMax,seedsPerAxis,obstacleFilename);
        rrt.SetStart(startpoint);
        rrt.SetGoal(endpoint);
        rrt.SetUpDataStructures();
        rrt.Run()
        x_path0=rrt.smoothedPath(:,1)';
        y_path0=rrt.smoothedPath(:,2)';
        z_path0=rrt.smoothedPath(:,3)';
        epsilon = 0.05; % Set a small epsilon value
        
        % Compute the differences between adjacent points
        dx = diff(x_path0);
        dy = diff(y_path0);
        dz = diff(z_path0);
        
        % Find indices of points to remove
        remove_indices = [false, (abs(dx) < epsilon) & (abs(dy) < epsilon) & (abs(dz) < epsilon)];
        
        % Remove points from the vectors
        x_path = x_path0(~remove_indices)';
        y_path = y_path0(~remove_indices)';
        z_path = z_path0(~remove_indices)';
        
        if x_path(end)~=x_path0(end)
            x_path(end)=x_path0(end); y_path(end)=y_path0(end);z_path(end)=z_path0(end);
        end
        % Display the results
        disp("x_path after removal:");
        disp(x_path);
        
        disp("y_path after removal:");
        disp(y_path);
        
        disp("z_path after removal:");
        disp(z_path);
        
        plot3(rrt.path(:,1),rrt.path(:,2),rrt.path(:,3),'k*'); 
        plot3(x_path,y_path,z_path,'k*'); hold on;
        dt = 0.25;
        t = (0:dt:15)';
        theta = t*pi/(2*t(end)); 
        tt=(1:size(x_path))/length(x_path)*pi/2;
        xx0 = interp1(tt,x_path,theta,'cubic');
        yy0 = interp1(tt,y_path,theta,'cubic');
        zz0 = interp1(tt,z_path,theta,'cubic');
        
        indx = ~isnan(xx0);
        xx1=xx0(indx);
        indy = ~isnan(yy0);
        yy1=yy0(indy);
        indz = ~isnan(zz0);
        zz1=zz0(indz);
        
        ttt=(1:size(xx1))/length(xx1)*pi/2;
        xx = interp1(ttt,xx1,theta,'linear','extrap');
        yy = interp1(ttt,yy1,theta,'linear','extrap');
        zz = interp1(ttt,zz1,theta,'linear','extrap');
        
        plot3(xx1,yy1,zz1,'r*')
        
        points=[xx,yy,zz];
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
        transformation_mat= [0,0,1;1,0,0;0,1,0];
        T_des(1:3,1:3) =transformation_mat*[rxx, -rxy,-rxz;-ryx,ryy,ryz;-rzx,rzy,rzz];     
        T_home(1:3,1:3) =transformation_mat*[rxx0, -rxy0,-rxz0;-ryx0,ryy0,ryz0;-rzx0,rzy0,rzz0];     
        [q_home,] = ik(eeName, T_home, weights0, initial_config); q_home=q_home'*180/pi;
        [q_des,] = ik(eeName, T_des, weights0, initial_config); q_des=q_des'*180/pi;
        figure (1)
        show(gen3,pi/180*q_home'); hold on;
        axis auto;
        view([45,0]);
        %% Define Path and Direction of Movement
        %Mind the relation between trajectory and time for there are vel limit
        pointsAR = [-yy+ay,zz-az,xx-ax];
        %upload photo and data to database
        jsonPoints=jsonencode(pointsAR);
        disp(jsonPoints);
        response = webwrite(url_to_HL,'json_points',jsonPoints);

        q=zeros(length(points),length(q_home)); q(:,1)=q_home(1):(q_des(1)-q_home(1))/(length(points)-1):q_des(1);
        q(:,2)=q_home(2):(q_des(2)-q_home(2))/(length(points)-1):q_des(2);q(:,3)=q_home(3):(q_des(3)-q_home(3))/(length(points)-1):q_des(3);
        q(:,4)=q_home(4):(q_des(4)-q_home(4))/(length(points)-1):q_des(4); q(:,5)=q_home(5):(q_des(5)-q_home(5))/(length(points)-1):q_des(5);
        q(:,6)=q_home(6):(q_des(6)-q_home(6))/(length(points)-1):q_des(6);q(:,7)=q_home(7):(q_des(7)-q_home(7))/(length(points)-1):q_des(7);
        hold on;   
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
        
        % Do You Run Kinova?
        previousid=currentid;
        while previousid==currentid
           apidecision = webread(decision);
           jsondecision=jsondecode(apidecision);
           if jsondecision.status=="true"
               currentid = str2num(jsondecision.id); %#ok<*ST2NM>  
           end
        end
        previousid=currentid;
        str=jsondecision.str;
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
