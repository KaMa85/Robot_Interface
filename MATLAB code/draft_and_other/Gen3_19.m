clear
clc
close all
%%
% Positional Transformation
ax=0.74;
ay=0.86;
az=-0.065;
gripper_state=2;
databaseip="http://localhost";
url_from_HL = databaseip+'/selectnewest.php';
url_to_HL = databaseip+'/insert.php';
url_to_HL_update = databaseip+'/insert5.php';
to_db = databaseip+'/insert4.php';
infor_to_db = databaseip+'/insert6.php';
decision = databaseip+'/selectdecision.php';
url_from_HL_moving_points = databaseip+'/selectnew_moving_point.php';
apiresponse_moving_points = webread(url_from_HL_moving_points);
jsonresponse_moving_points=jsondecode(apiresponse_moving_points);
if jsonresponse_moving_points.status=="true"
    current_id_moving_points = str2num(jsonresponse_moving_points.id); 
    last_id_moving_points=current_id_moving_points;
else
    response = webwrite(infor_to_db,'DB','y');
    error("Not connected to the database");
end

apiresponse = webread(url_from_HL);
jsonresponse=jsondecode(apiresponse);
if jsonresponse.status=="true"
    currentvideoid = str2num(jsonresponse.id); %#ok<*ST2NM>  
    previouslyknownid=currentvideoid;
else
    error("Not connected to the database");
end

apidecision = webread(decision);
jsondecision=jsondecode(apidecision);
if jsondecision.status=="true"
    currentid = str2num(jsondecision.id); %#ok<*ST2NM>  
    previousid=currentid;
else
    error("Not connected to the database");
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
%%
while true
    %%%%connection to the data 
    while previouslyknownid < currentvideoid
        close all;
        apiresponse =  webread(url_from_HL);
        jsonresponse=jsondecode(apiresponse);
        if jsonresponse.status=="true"
            gripper_state=str2num(jsonresponse.EE_state);
            if gripper_state==1
                gripper_state=2;
                response = webwrite(to_db,'EE',2);  
                toolCommand = int32(2);  
                toolCmd = 1;
                toolDuration = 0;
                isOk = gen3Kinova.SendToolCommand(toolCommand, toolDuration, toolCmd);
                if isOk
                    disp('Command sent to the gripper. Wait for the gripper to open.')
                else
                    error('Command Error.');
                end
                apiresponse = webread(url_from_HL);
                jsonresponse=jsondecode(apiresponse);
                if jsonresponse.status=="true"
                    currentvideoid = str2num(jsonresponse.id); %#ok<*ST2NM>  
                    previouslyknownid=currentvideoid;
                else
                    error("Not connected to the database");
                end
                break;
            elseif gripper_state==0
                gripper_state=2;
                response = webwrite(to_db,'EE',2); 
                toolCommand = int32(2);    % Velocity control mode
                toolDuration = 0;
                toolCmd = -1; % Close the gripper with full speed
                isOk = gen3Kinova.SendToolCommand(toolCommand, toolDuration, toolCmd);
                if isOk
                    disp('Command sent to the gripper. Wait for the gripper to close.')
                else
                    error('Command Error.');
                end
                apiresponse = webread(url_from_HL);
                jsonresponse=jsondecode(apiresponse);
                if jsonresponse.status=="true"
                    currentvideoid = str2num(jsonresponse.id); %#ok<*ST2NM>  
                    previouslyknownid=currentvideoid;
                else
                    error("Not connected to the database");
                end
                break;
            end
            if str2num(jsonresponse.lr_z)>str2num(jsonresponse.ll_z)
                lrx=str2num(jsonresponse.lr_z)+0.14+ax;lry=-str2num(jsonresponse.lr_x)+ay;lrz=str2num(jsonresponse.lr_y)-0.11+az;
                llx=str2num(jsonresponse.ll_z)-0.14+ax;lly=-str2num(jsonresponse.ll_x)+ay;llz=str2num(jsonresponse.ll_y)-0.11+az;
                urx=str2num(jsonresponse.ur_z)+0.14+ax;ury=-str2num(jsonresponse.ur_x)+ay;urz=str2num(jsonresponse.ur_y)+0.07+az;
                ulx=str2num(jsonresponse.ul_z)-0.14+ax;uly=-str2num(jsonresponse.ul_x)+ay;ulz=str2num(jsonresponse.ul_y)+0.07+az;
            else
                lrx=str2num(jsonresponse.lr_z)-0.14+ax;lry=-str2num(jsonresponse.lr_x)+ay;lrz=str2num(jsonresponse.lr_y)-0.11+az;
                llx=str2num(jsonresponse.ll_z)+0.14+ax;lly=-str2num(jsonresponse.ll_x)+ay;llz=str2num(jsonresponse.ll_y)-0.11+az;
                urx=str2num(jsonresponse.ur_z)-0.14+ax;ury=-str2num(jsonresponse.ur_x)+ay;urz=str2num(jsonresponse.ur_y)+0.07+az;
                ulx=str2num(jsonresponse.ul_z)+0.14+ax;uly=-str2num(jsonresponse.ul_x)+ay;ulz=str2num(jsonresponse.ul_y)+0.07+az;
            end
            lr = [lrx, lry,lrz];ll = [llx, lly,llz];ur = [urx, ury,urz];ul = [ulx, uly,ulz];
            constr_from_AR=[lr;ur;ul;ll];
            writematrix(constr_from_AR,'constr','Delimiter','tab')
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
            error('Not Connected to the Database');
        end
        pause(0.1);
        %% path planning
        % initialize rrt
        constr=load('constr.txt');
        obstacleFilename = 'constr.txt';
        rrt = RrtPlanner(25,3,1,constr);
        rrt.SetStart(startpoint);
        rrt.SetGoal(endpoint);
        rrt.SetUpDataStructures();
        rrt.Run()
        delete(rrt);
        % final rrt
        seedsPerAxis = 7;
        treesMax = seedsPerAxis^3*3+2;
        rrt = RrtPlanner(treesMax,seedsPerAxis,obstacleFilename,constr);
        rrt.drawingSkipsPerDrawing = 30;
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
        dt = 0.25;
        t = (0:dt:25)';
        theta = t*pi/(2*t(end)); 
        tt = (1:length(x_path))/length(x_path)*pi/2;
        xx0 = interp1(tt, x_path, theta, 'cubic');
        yy0 = interp1(tt, y_path, theta, 'cubic');
        zz0 = interp1(tt, z_path, theta, 'cubic');  
        % Remove NaN values
        validIndices = ~isnan(xx0) & ~isnan(yy0) & ~isnan(zz0);
        xx0_clean = xx0(validIndices);
        yy0_clean = yy0(validIndices);
        zz0_clean = zz0(validIndices);
        % Calculate the differences between consecutive points
        dx = diff(xx0_clean);
        dy = diff(yy0_clean);
        dz = diff(zz0_clean);      
        % Calculate the cumulative arc length
        arcLen = cumsum(sqrt(dx.^2 + dy.^2 + dz.^2));
        arcLength = [0; arcLen];  % Include the starting point    
        % Normalize the arc length to create a parameter s that varies from 0 to 1
        s = arcLength / arcLength(end);       
        % Create a non-linear distribution for s_uniform
        s_uniform = linspace(0, 1, length(theta))';
        s_uniform = 0.5 - 0.5 * cos(pi * s_uniform);  % Adjust this line as needed
        % Interpolate at the non-uniformly spaced values of s
        xx = interp1(s, xx0_clean, s_uniform, 'linear', 'extrap');
        yy = interp1(s, yy0_clean, s_uniform, 'linear', 'extrap');
        zz = interp1(s, zz0_clean, s_uniform, 'linear', 'extrap');
        points=[xx,yy,zz];
        %% Define and Show Robot at Initial State (Home)
        gen3 = loadrobot("kinovaGen3");%Type 'rigidBodyTree'
        gen3.DataFormat = 'column'; 
        eeName = 'EndEffector_Link';
        names=gen3.BodyNames;
        ik = inverseKinematics('RigidBodyTree', gen3);
        ik.SolverParameters.AllowRandomRestart = false;
        initial_config = gen3.homeConfiguration;
        weights = [1, 1, 1, 1, 1, 1];
        numWaypoints = size(points,1);
        T_home_ref = getTransform(gen3, pi*initial_config/180, eeName);
        T_des_ref = getTransform(gen3, pi*initial_config/180, eeName);
        %transformation_mat=eye(3);
        transformation_mat= [0,0,1;1,0,0;0,1,0];        
        T_home_ref(1:3,1:3) =transformation_mat*[rxx0, -rxy0,-rxz0;-ryx0,ryy0,ryz0;-rzx0,rzy0,rzz0];  
        T_home_ref(1:3,4)=[xx(1);yy(1);zz(1)];
        T_des_ref(1:3,1:3) =transformation_mat*[rxx, -rxy,-rxz;-ryx,ryy,ryz;-rzx,rzy,rzz];
        T_home_ref(1:3,4)=[xx(end);yy(end);zz(end)];
        [q_home_ref,] = ik(eeName, T_home_ref, weights, initial_config); q_home_ref=q_home_ref*180/pi;
        [q_des_ref,] = ik(eeName, T_des_ref, weights, initial_config); q_des_ref=q_des_ref'*180/pi;
        quat_home = rotm2quat(T_home_ref(1:3,1:3));
        quat_des = rotm2quat(T_des_ref(1:3,1:3));
        quats = zeros(numWaypoints, 4);
        for i = 1:numWaypoints
            s = (i - 1) / (numWaypoints - 1); % Interpolation factor [0, 1]
            quats(i,:) = quatinterp(quat_home, quat_des, s, 'slerp');
        end
        %% Define Path and Direction of Movement
        %Points in AR coordinatesystem 
        pointsAR = [-yy+ay,zz-az,xx-ax];
        %upload data to database
        jsonPoints=jsonencode(pointsAR);
        response = webwrite(url_to_HL,'json_points',jsonPoints); 
    %% Compute Joint Angle Based on Inverse Kinematics of the Path and Animate
        q_init = q_home_ref;
        numJoints = size(pi/180*q_home_ref,1);   
        qs = zeros(numWaypoints,numJoints);
        for i = 1:numWaypoints
            T_des = eye(4);
            T_des(1:3, 4) = points(i,:)'; % Update position from trajectory
            T_des(1:3,1:3) = quat2rotm(quats(i,:)); % Update orientation from interpolated quaternions
            [q_sol, q_info] = ik(eeName, T_des, weights, q_init);
            %q_info
            % Store the configuration
            if isempty(q_sol) || q_info.Status~="success"
                response = webwrite(infor_to_db,'IK','y'); %#ok<NASGU> 
                disp(['Failed to compute IK for waypoint ', num2str(i)]); % Optional: Display a message
                pause(5);
                response = webwrite(infor_to_db,'IK','n');
                break;
            end
            qs(i,:) = q_sol(1:numJoints); 
            % Start from prior solution
            q_init = q_sol;
        end
        % Do You Run Kinova?
        if q_info.Status~="success"
            str = 'n';
        else
            while previousid==currentid
                apidecision = webread(decision);
                jsondecision=jsondecode(apidecision);
                if jsondecision.status=="true"
                    currentid = str2num(jsondecision.id); %#ok<*ST2NM>  
                end
            end
            previousid=currentid;
            str=jsondecision.str;
        end
        if str ~= 'y'
            disp('Operation halted')
            close all;
            apiresponse = webread(url_from_HL);
            jsonresponse=jsondecode(apiresponse);
            if jsonresponse.status=="true"
                currentvideoid = str2num(jsonresponse.id); %#ok<*ST2NM>  
                previouslyknownid=currentvideoid;
            else
                error("Not connected to the database");
            end
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
            error('SendJointAngles cmd error');
        end
        pause(0.1)
        %Check if the robot has reached the starting position
        while 1
            if (last_id_moving_points==current_id_moving_points)
                apiresponse_moving_points = webread(url_from_HL_moving_points);
                jsonresponse_moving_points=jsondecode(apiresponse_moving_points);
                if jsonresponse_moving_points.status=="true"
                    current_id_moving_points = str2num(jsonresponse_moving_points.id);
                end
            else
                point=getTransform(gen3, pi*actuatorFb.position'/180, eeName);
                point=point(1:3,4)';
                differences = bsxfun(@minus, points, point);
                squaredDifferences = sum(differences .^ 2, 2);                
                % Find the index of the smallest distance
                [~, closestIndex] = min(squaredDifferences);                
                apiresponse_moving_points = webread(url_from_HL_moving_points);
                jsonresponse_moving_points=jsondecode(apiresponse_moving_points);
                current_id_moving_points = str2num(jsonresponse_moving_points.id);
                run_moving_path= parfeval(backgroundPool, @moving_path, 3,ax,ay,az, constr, points(closestIndex+10,:), points(end,:), xx , yy , zz, jsonresponse_moving_points.new_path);
                if run_moving_path.State =="queued" || run_moving_path.State =="finished"
                    [x_moving_path, y_moving_path,z_moving_path] = fetchOutputs(run_moving_path);
                    %Points in AR coordinatesystem 
                    pointsAR_moving = [-y_moving_path+ay,z_moving_path-az, x_moving_path-ax];
                    %upload data to database
                    jsonPoints_moving=jsonencode(pointsAR_moving);
                    response = webwrite(url_to_HL_update,'json_points',jsonPoints_moving);
                    pause(0.35);
                    if jsonresponse_moving_points.status=="true"
                        current_id_moving_points = str2num(jsonresponse_moving_points.id);
                        apiresponse_moving_points = webread(url_from_HL_moving_points);
                        jsonresponse_moving_points=jsondecode(apiresponse_moving_points);
                        last_id_moving_points=current_id_moving_points;
                    end
                else
                        error("Not connected to the database");
                end
            end
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
            if (last_id_moving_points==current_id_moving_points)
                apiresponse_moving_points = webread(url_from_HL_moving_points);
                jsonresponse_moving_points=jsondecode(apiresponse_moving_points);
                if jsonresponse_moving_points.status=="true"
                    current_id_moving_points = str2num(jsonresponse_moving_points.id);
                end
            else
                point=getTransform(gen3, pi*actuatorFb.position'/180, eeName);
                point=point(1:3,4)';
                % Calculate the squared Euclidean distances
                differences = bsxfun(@minus, points, point);
                squaredDifferences = sum(differences .^ 2, 2);
                
                % Find the index of the smallest distance
                [~, closestIndex] = min(squaredDifferences);
                apiresponse_moving_points = webread(url_from_HL_moving_points);
                jsonresponse_moving_points=jsondecode(apiresponse_moving_points);
                run_moving_path= parfeval(backgroundPool, @moving_path, 3 , ax , ay , az , constr , points(closestIndex+10,:), points(end,:), xx , yy , zz, jsonresponse_moving_points.new_path);
                if run_moving_path.State =="queued" || run_moving_path.State =="finished"
                    [x_moving_path, y_moving_path,z_moving_path] = fetchOutputs(run_moving_path);
                    %Points in AR coordinatesystem 
                    pointsAR_moving = [-y_moving_path+ay,z_moving_path-az, x_moving_path-ax];
                    %upload data to database
                    jsonPoints_moving=jsonencode(pointsAR_moving);
                    response = webwrite(url_to_HL_update,'json_points',jsonPoints_moving);
                    pause(0.35);
                    if jsonresponse_moving_points.status=="true"
                        current_id_moving_points = str2num(jsonresponse_moving_points.id);
                        apiresponse_moving_points = webread(url_from_HL_moving_points);
                        jsonresponse_moving_points=jsondecode(apiresponse_moving_points);
                        last_id_moving_points=current_id_moving_points;
                    end
                else
                        disp("Not connected to the database");
                end
            end
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
        apiresponse = webread(url_from_HL);
        jsonresponse=jsondecode(apiresponse);
        if jsonresponse.status=="true"
            currentvideoid = str2num(jsonresponse.id); %#ok<*ST2NM>  
            previouslyknownid=currentvideoid;
        else
            error("Not connected to the database");
        end
    end
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
