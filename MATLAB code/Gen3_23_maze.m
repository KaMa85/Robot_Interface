clear
clc
close all
%%
% Positional Transformation
ax=0.73;
ay=0.85;
az=-0.065;
gripper_state=2;
break_main_loop=false;
auto_struct = struct('first_gripper_action', [], 'first_motion', [], 'second_gripper_action', [], 'second_motion', []);
fields = fieldnames(auto_struct);
% options = weboptions('MediaType','application/json');
databaseip="http://localhost";
url_from_HL = databaseip+'/selectnewest.php';
url_to_HL = databaseip+'/insert.php';
url_to_HL_update = databaseip+'/insert5.php';
to_db = databaseip+'/insert4.php';
infor_to_db = databaseip+'/insert6.php';
auto_to_db=databaseip+'/insert7.php';
decision = databaseip+'/selectdecision.php';
automation = databaseip+'/selectautomation.php';
url_from_HL_moving_points = databaseip+'/selectnew_moving_point.php';
% Updating the database id for moving point

tic
while true
    apiresponse_moving_points = webread(url_from_HL_moving_points);
    jsonresponse_moving_points=jsondecode(apiresponse_moving_points);
    if jsonresponse_moving_points.status=="true"
        current_id_moving_points = str2num(jsonresponse_moving_points.id); 
        last_id_moving_points=current_id_moving_points;
        disp("Connected");
        break;
    elseif toc>3  
        error("Not connected to the database");
    else
        disp("Connecting to the database");
        pause(1);
    end
end
% Updating the database id for new transform (main loop)
tic
while true
    apiresponse = webread(url_from_HL);
    jsonresponse=jsondecode(apiresponse);
    if jsonresponse.status=="true"
        currentvideoid = str2num(jsonresponse.id); %#ok<*ST2NM>  
        previouslyknownid=currentvideoid;
        break;
    elseif toc>3  
        error("Not connected to the database");
    else
        disp("Connecting to the database");
        pause(1);
    end
end
% Updating the database for automation
webwrite(auto_to_db,'autom',0);
% Updating the database id for decision
tic
while true
    apidecision = webread(decision);
    jsondecision=jsondecode(apidecision);
    if jsondecision.status=="true"
        currentid = str2num(jsondecision.id); %#ok<*ST2NM>  
        previousid=currentid;
        break;
    elseif toc>3  
        error("Not connected to the database");
    else
        disp("Connecting to the database");
        pause(1);
    end
end
% Connecting to robot
tic;
while true
    Simulink.importExternalCTypes(which('kortex_wrapper_data.h'));
    gen3Kinova = kortex();
    %gen3Kinova.ip_address = '172.31.99.235';
    gen3Kinova.ip_address = '192.168.1.10'; %Manual ip address
    gen3Kinova.user = 'admin';
    gen3Kinova.password = 'admin';
    isOk = gen3Kinova.CreateRobotApisWrapper();
    if isOk
       disp('You are connected to the robot!'); 
       break;
    else
        webwrite(infor_to_db, jsonencode(struct('robo', 'y', 'motion', 'n', 'IK', 'n')));
        pause(5);
        webwrite(infor_to_db, jsonencode(struct('robo', 'n', 'motion', 'n', 'IK', 'n')));
        disp('Failed to connect to the robot. Still trying to connect!');
        if toc>120
            error('Failed to establish a valid connection!');
        end
    end
end
ind=1;
%%
while true
        apiautomation = webread(automation);
        jsonautomation=jsondecode(apiautomation);
        if jsonautomation.status=="true"
            auto_bool=str2num(jsonautomation.auto);
            if auto_bool==1
                for i = 1:length(fields)
                     auto_struct.(fields{i}) = [];
                end
                disp('Automation structure is cleared')
                webwrite(auto_to_db,'autom',0);
            end
        end
    %%%%connection to the data 
    while previouslyknownid < currentvideoid
        close all;
        apiresponse =  webread(url_from_HL);
        jsonresponse=jsondecode(apiresponse);
        if jsonresponse.status=="true"
            gripper_state=str2num(jsonresponse.EE_state);
            if gripper_state==1
                auto_struct.first_gripper_action=-0.4;
                auto_struct.second_gripper_action=1;
                gripper_state=2;
                webwrite(to_db,'EE',2);  
                toolCommand = int32(2);  
                toolCmd = 1;
                toolDuration = 0;
                isOk = gen3Kinova.SendToolCommand(toolCommand, toolDuration, toolCmd);
                if isOk
                    disp('Command sent to the gripper. Wait for the gripper to open.')
                else
                    webwrite(infor_to_db, jsonencode(struct('robo', 'n', 'motion', 'y', 'IK', 'n')));
                    pause(5);
                    webwrite(infor_to_db, jsonencode(struct('robo', 'n', 'motion', 'n', 'IK', 'n')));
                    disp('Command Error.');
                end
                tic
                while true
                    apiresponse = webread(url_from_HL);
                    jsonresponse=jsondecode(apiresponse);
                    if jsonresponse.status=="true"
                        currentvideoid = str2num(jsonresponse.id); %#ok<*ST2NM>  
                        previouslyknownid=currentvideoid;
                        break;
                    elseif toc>3  
                        error("Not connected to the database");
                    else
                        disp("Connecting to the database");
                        pause(1);
                    end
                end
                break;
            elseif gripper_state==0        
                auto_struct.first_gripper_action=1;
                auto_struct.second_gripper_action=-0.4;               
                gripper_state=2;
                webwrite(to_db,'EE',2); 
                toolCommand = int32(2);    % Velocity control mode
                toolDuration = 0;
                toolCmd = -0.4; % Close the gripper with full speed
                isOk = gen3Kinova.SendToolCommand(toolCommand, toolDuration, toolCmd);
                if isOk
                    disp('Command sent to the gripper. Wait for the gripper to close.')
                else
                    webwrite(infor_to_db, jsonencode(struct('robo', 'n', 'motion', 'y', 'IK', 'n')));
                    pause(5);
                    webwrite(infor_to_db, jsonencode(struct('robo', 'n', 'motion', 'n', 'IK', 'n')));
                    disp('Command Error.');
                end
                tic
                while true
                    apiresponse = webread(url_from_HL);
                    jsonresponse=jsondecode(apiresponse);
                    if jsonresponse.status=="true"
                        currentvideoid = str2num(jsonresponse.id); %#ok<*ST2NM>  
                        previouslyknownid=currentvideoid;
                        break;
                    elseif toc>3  
                        error("Not connected to the database");
                    else
                        disp("Connecting to the database");
                        pause(1);
                    end
                end
                break;
            end
            if str2num(jsonresponse.lr_z)>str2num(jsonresponse.ll_z)
                lrx=str2num(jsonresponse.lr_z)+0.14+ax;lry=-str2num(jsonresponse.lr_x)+ay;lrz=str2num(jsonresponse.lr_y)-0.20+az;
                llx=str2num(jsonresponse.ll_z)-0.14+ax;lly=-str2num(jsonresponse.ll_x)+ay;llz=str2num(jsonresponse.ll_y)-0.20+az;
                urx=str2num(jsonresponse.ur_z)+0.14+ax;ury=-str2num(jsonresponse.ur_x)+ay;urz=str2num(jsonresponse.ur_y)+0.2+az;
                ulx=str2num(jsonresponse.ul_z)-0.14+ax;uly=-str2num(jsonresponse.ul_x)+ay;ulz=str2num(jsonresponse.ul_y)+0.2+az;
                lrx2=str2num(jsonresponse.lr_z2)+0.14+ax;lry2=-str2num(jsonresponse.lr_x2)+ay;lrz2=str2num(jsonresponse.lr_y2)-0.20+az;
                llx2=str2num(jsonresponse.ll_z2)-0.14+ax;lly2=-str2num(jsonresponse.ll_x2)+ay;llz2=str2num(jsonresponse.ll_y2)-0.20+az;
                urx2=str2num(jsonresponse.ur_z2)+0.14+ax;ury2=-str2num(jsonresponse.ur_x2)+ay;urz2=str2num(jsonresponse.ur_y2)+0.2+az;
                ulx2=str2num(jsonresponse.ul_z2)-0.14+ax;uly2=-str2num(jsonresponse.ul_x2)+ay;ulz2=str2num(jsonresponse.ul_y2)+0.2+az;
                lrx3=str2num(jsonresponse.lr_z3)+0.14+ax;lry3=-str2num(jsonresponse.lr_x3)+ay;lrz3=str2num(jsonresponse.lr_y3)-0.20+az;
                llx3=str2num(jsonresponse.ll_z3)-0.14+ax;lly3=-str2num(jsonresponse.ll_x3)+ay;llz3=str2num(jsonresponse.ll_y3)-0.20+az;
                urx3=str2num(jsonresponse.ur_z3)+0.14+ax;ury3=-str2num(jsonresponse.ur_x3)+ay;urz3=str2num(jsonresponse.ur_y3)+0.2+az;
                ulx3=str2num(jsonresponse.ul_z3)-0.14+ax;uly3=-str2num(jsonresponse.ul_x3)+ay;ulz3=str2num(jsonresponse.ul_y3)+0.2+az;
                lrx4=str2num(jsonresponse.lr_z4)+0.14+ax;lry4=-str2num(jsonresponse.lr_x4)+ay;lrz4=str2num(jsonresponse.lr_y4)-0.20+az;
                llx4=str2num(jsonresponse.ll_z4)-0.14+ax;lly4=-str2num(jsonresponse.ll_x4)+ay;llz4=str2num(jsonresponse.ll_y4)-0.20+az;
                urx4=str2num(jsonresponse.ur_z4)+0.14+ax;ury4=-str2num(jsonresponse.ur_x4)+ay;urz4=str2num(jsonresponse.ur_y4)+0.2+az;
                ulx4=str2num(jsonresponse.ul_z4)-0.14+ax;uly4=-str2num(jsonresponse.ul_x4)+ay;ulz4=str2num(jsonresponse.ul_y4)+0.2+az;
                lrx5=str2num(jsonresponse.lr_z5)+0.14+ax;lry5=-str2num(jsonresponse.lr_x5)+ay;lrz5=str2num(jsonresponse.lr_y5)-0.20+az;
                llx5=str2num(jsonresponse.ll_z5)-0.14+ax;lly5=-str2num(jsonresponse.ll_x5)+ay;llz5=str2num(jsonresponse.ll_y5)-0.20+az;
                urx5=str2num(jsonresponse.ur_z5)+0.14+ax;ury5=-str2num(jsonresponse.ur_x5)+ay;urz5=str2num(jsonresponse.ur_y5)+0.2+az;
                ulx5=str2num(jsonresponse.ul_z5)-0.14+ax;uly5=-str2num(jsonresponse.ul_x5)+ay;ulz5=str2num(jsonresponse.ul_y5)+0.2+az;
                lrx6=str2num(jsonresponse.lr_z6)+0.14+ax;lry6=-str2num(jsonresponse.lr_x6)+ay;lrz6=str2num(jsonresponse.lr_y6)-0.20+az;
                llx6=str2num(jsonresponse.ll_z6)-0.14+ax;lly6=-str2num(jsonresponse.ll_x6)+ay;llz6=str2num(jsonresponse.ll_y6)-0.20+az;
                urx6=str2num(jsonresponse.ur_z6)+0.14+ax;ury6=-str2num(jsonresponse.ur_x6)+ay;urz6=str2num(jsonresponse.ur_y6)+0.2+az;
                ulx6=str2num(jsonresponse.ul_z6)-0.14+ax;uly6=-str2num(jsonresponse.ul_x6)+ay;ulz6=str2num(jsonresponse.ul_y6)+0.2+az;              
            else
                lrx=str2num(jsonresponse.lr_z)-0.14+ax;lry=-str2num(jsonresponse.lr_x)+ay;lrz=str2num(jsonresponse.lr_y)-0.20+az;
                llx=str2num(jsonresponse.ll_z)+0.14+ax;lly=-str2num(jsonresponse.ll_x)+ay;llz=str2num(jsonresponse.ll_y)-0.20+az;
                urx=str2num(jsonresponse.ur_z)-0.14+ax;ury=-str2num(jsonresponse.ur_x)+ay;urz=str2num(jsonresponse.ur_y)+0.2+az;
                ulx=str2num(jsonresponse.ul_z)+0.14+ax;uly=-str2num(jsonresponse.ul_x)+ay;ulz=str2num(jsonresponse.ul_y)+0.2+az;
                lrx2=str2num(jsonresponse.lr_z2)-0.14+ax;lry2=-str2num(jsonresponse.lr_x2)+ay;lrz2=str2num(jsonresponse.lr_y2)-0.20+az;
                llx2=str2num(jsonresponse.ll_z2)+0.14+ax;lly2=-str2num(jsonresponse.ll_x2)+ay;llz2=str2num(jsonresponse.ll_y2)-0.20+az;
                urx2=str2num(jsonresponse.ur_z2)-0.14+ax;ury2=-str2num(jsonresponse.ur_x2)+ay;urz2=str2num(jsonresponse.ur_y2)+0.2+az;
                ulx2=str2num(jsonresponse.ul_z2)+0.14+ax;uly2=-str2num(jsonresponse.ul_x2)+ay;ulz2=str2num(jsonresponse.ul_y2)+0.2+az;
                lrx3=str2num(jsonresponse.lr_z3)-0.14+ax;lry3=-str2num(jsonresponse.lr_x3)+ay;lrz3=str2num(jsonresponse.lr_y3)-0.20+az;
                llx3=str2num(jsonresponse.ll_z3)+0.14+ax;lly3=-str2num(jsonresponse.ll_x3)+ay;llz3=str2num(jsonresponse.ll_y3)-0.20+az;
                urx3=str2num(jsonresponse.ur_z3)-0.14+ax;ury3=-str2num(jsonresponse.ur_x3)+ay;urz3=str2num(jsonresponse.ur_y3)+0.2+az;
                ulx3=str2num(jsonresponse.ul_z3)+0.14+ax;uly3=-str2num(jsonresponse.ul_x3)+ay;ulz3=str2num(jsonresponse.ul_y3)+0.2+az;            
                lrx4=str2num(jsonresponse.lr_z4)-0.14+ax;lry4=-str2num(jsonresponse.lr_x4)+ay;lrz4=str2num(jsonresponse.lr_y4)-0.20+az;
                llx4=str2num(jsonresponse.ll_z4)+0.14+ax;lly4=-str2num(jsonresponse.ll_x4)+ay;llz4=str2num(jsonresponse.ll_y4)-0.20+az;
                urx4=str2num(jsonresponse.ur_z4)-0.14+ax;ury4=-str2num(jsonresponse.ur_x4)+ay;urz4=str2num(jsonresponse.ur_y4)+0.2+az;
                ulx4=str2num(jsonresponse.ul_z4)+0.14+ax;uly4=-str2num(jsonresponse.ul_x4)+ay;ulz4=str2num(jsonresponse.ul_y4)+0.2+az;                        
                lrx5=str2num(jsonresponse.lr_z5)-0.14+ax;lry5=-str2num(jsonresponse.lr_x5)+ay;lrz5=str2num(jsonresponse.lr_y5)-0.20+az;
                llx5=str2num(jsonresponse.ll_z5)+0.14+ax;lly5=-str2num(jsonresponse.ll_x5)+ay;llz5=str2num(jsonresponse.ll_y5)-0.20+az;
                urx5=str2num(jsonresponse.ur_z5)-0.14+ax;ury5=-str2num(jsonresponse.ur_x5)+ay;urz5=str2num(jsonresponse.ur_y5)+0.2+az;
                ulx5=str2num(jsonresponse.ul_z5)+0.14+ax;uly5=-str2num(jsonresponse.ul_x5)+ay;ulz5=str2num(jsonresponse.ul_y5)+0.2+az;
                lrx6=str2num(jsonresponse.lr_z6)-0.14+ax;lry6=-str2num(jsonresponse.lr_x6)+ay;lrz6=str2num(jsonresponse.lr_y6)-0.20+az;
                llx6=str2num(jsonresponse.ll_z6)+0.14+ax;lly6=-str2num(jsonresponse.ll_x6)+ay;llz6=str2num(jsonresponse.ll_y6)-0.20+az;
                urx6=str2num(jsonresponse.ur_z6)-0.14+ax;ury6=-str2num(jsonresponse.ur_x6)+ay;urz6=str2num(jsonresponse.ur_y6)+0.2+az;
                ulx6=str2num(jsonresponse.ul_z6)+0.14+ax;uly6=-str2num(jsonresponse.ul_x6)+ay;ulz6=str2num(jsonresponse.ul_y6)+0.2+az;                

            end 

            % Writing to obstacles as arrays
            constr_from_AR = [lrx, lry, lrz; urx, ury, urz ; ulx, uly, ulz ; llx, lly, llz ];
            constr_from_AR2 = [lrx2, lry2, lrz2; urx2, ury2, urz2; ulx2, uly2, ulz2 ; llx2, lly2, llz2 ];
            constr_from_AR3 = [lrx3, lry3, lrz3; urx3, ury3, urz3; ulx3, uly3, ulz3 ; llx3, lly3, llz3];
            constr_from_AR4 = [lrx4, lry4, lrz4; urx4, ury4, urz4; ulx4, uly4, ulz4; llx4, lly4, llz4];
            constr_from_AR5 = [lrx5, lry5, lrz5; urx5, ury5, urz5; ulx5, uly5, ulz5; llx5, lly5, llz5];
            constr_from_AR6 = [lrx6, lry6, lrz6; urx6, ury6, urz6; ulx6, uly6, ulz6; llx6, lly6, llz6];
            % Writing to constr.txt as specified
            writematrix(constr_from_AR, 'constr.txt', 'Delimiter', 'tab');
            
            % Append the rest with a newline in between each
            fileID = fopen('constr.txt', 'a');
            fprintf(fileID, '\r\n');
            fclose(fileID);
            writematrix(constr_from_AR2, 'constr.txt', 'Delimiter', 'tab', 'WriteMode', 'append');
            
            fileID = fopen('constr.txt', 'a');
            fprintf(fileID, '\r\n');
            fclose(fileID);
            writematrix(constr_from_AR3, 'constr.txt', 'Delimiter', 'tab', 'WriteMode', 'append');
            
            fileID = fopen('constr.txt', 'a');
            fprintf(fileID, '\r\n');
            fclose(fileID);
            writematrix(constr_from_AR4, 'constr.txt', 'Delimiter', 'tab', 'WriteMode', 'append');
            
            fileID = fopen('constr.txt', 'a');
            fprintf(fileID, '\r\n');
            fclose(fileID);
            writematrix(constr_from_AR5, 'constr.txt', 'Delimiter', 'tab', 'WriteMode', 'append');
            
            fileID = fopen('constr.txt', 'a');
            fprintf(fileID, '\r\n');
            fclose(fileID);
            writematrix(constr_from_AR6, 'constr.txt', 'Delimiter', 'tab', 'WriteMode', 'append');

            % Append the third array to the file
            writematrix(constr_from_AR3, 'constr.txt', 'Delimiter', 'tab', 'WriteMode', 'append');
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
            tic
            while true
                apiresponse = webread(url_from_HL);
                jsonresponse=jsondecode(apiresponse);
                if jsonresponse.status=="true"
                    currentvideoid = str2num(jsonresponse.id); %#ok<*ST2NM>  
                    previouslyknownid=currentvideoid;
                    break;
                elseif toc>3  
                    error("Not connected to the database");
                else
                    disp("Connecting to the database");
                    pause(1);
                end
            end
            break;
        end
        pause(0.1);
        %% path planning
        % Initialize variables for storing distinct paths
        distinctPaths = {};
        dt = 0.25;
        t = (0:dt:25)';
        epsilon = 0.05; % Small value for filtering close points
        for i = 1:7
            % Load constraints
            constr = load('constr.txt');
            obstacleFilename = 'constr.txt';
        
            % First run of RRT
            rrt = RrtPlanner(25, 3, 1, constr);
            rrt.SetStart(startpoint);
            rrt.SetGoal(endpoint);
            rrt.SetUpDataStructures();
            rrt.Run();
            delete(rrt);
            % Final RRT with more trees
            seedsPerAxis = 7;
            treesMax = seedsPerAxis^3*3+2;
            rrt = RrtPlanner(treesMax, seedsPerAxis, obstacleFilename, constr);
            rrt.drawingSkipsPerDrawing = 30;
            rrt.SetStart(startpoint);
            rrt.SetGoal(endpoint);
            rrt.SetUpDataStructures();
            rrt.Run();
            % Extract and clean path
            [x_path, y_path, z_path] = cleanPath(rrt.smoothedPath, epsilon);
        
            % Interpolate and normalize path
            [xx, yy, zz] = interpolateAndNormalizePath(x_path, y_path, z_path);
        
            % Create a temporary variable for the current 'points'
            currentPoints = [xx, yy, zz];
        
            % Check if the current 'points' are distinct
            if ~isPathDuplicate(currentPoints, distinctPaths, 0.0005)
                distinctPaths{end+1} = currentPoints; %#ok<SAGROW> 
            end
        end
        xxx = [];
        yyy = [];
        zzz = [];
        
        for i = 1:length(distinctPaths)
            xxx = [xxx; distinctPaths{i}(:,1)]; %#ok<AGROW> 
            yyy = [yyy; distinctPaths{i}(:,2)]; %#ok<AGROW> 
            zzz = [zzz; distinctPaths{i}(:,3)]; %#ok<AGROW> 
        end
        distincts_together=[xxx,yyy,zzz];
        %% Sending path to  the AR headset
        %Points in AR coordinatesystem 
        pointsAR = [-yyy+ay,zzz-az,xxx-ax];
        %upload data to database
        jsonPoints=jsonencode(pointsAR);
        webwrite(url_to_HL,'json_points',jsonPoints); 
        %% Waiting for the user to pick up a point in one of the distinct paths 
        while 1 
            if (last_id_moving_points==current_id_moving_points)
                    apiresponse_moving_points = webread(url_from_HL_moving_points);
                    jsonresponse_moving_points=jsondecode(apiresponse_moving_points);
                    if jsonresponse_moving_points.status=="true"
                        current_id_moving_points = str2num(jsonresponse_moving_points.id);
                    end
            else
                apiresponse_moving_points = webread(url_from_HL_moving_points);
                jsonresponse_moving_points=jsondecode(apiresponse_moving_points);
                current_id_moving_points = str2num(jsonresponse_moving_points.id);
                last_id_moving_points=current_id_moving_points;
                points=which_moved(distincts_together,length(distinctPaths), jsonresponse_moving_points.new_path,ax,ay,az);
                % Sending path to  the AR headset
                pointsAR = [-points(:,2)+ay,points(:,3)-az,points(:,1)-ax];
                %upload data to database
                jsonPoints=jsonencode(pointsAR);
                webwrite(url_to_HL_update,'json_points',jsonPoints); 
                break
            end
        end
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
                    webwrite(infor_to_db, jsonencode(struct('robo', 'n', 'motion', 'n', 'IK', 'y')));
                disp(['Failed to compute IK for waypoint ', num2str(i)]); % Optional: Display a message
                pause(5);
                    webwrite(infor_to_db, jsonencode(struct('robo', 'n', 'motion', 'n', 'IK', 'n')));
                break;
            end
            qs(i,:) = q_sol(1:numJoints); 
            % Start from prior solution
            q_init = q_sol;
        end
        figure (3)
        framesPerSecond = 30;
        r = robotics.Rate(framesPerSecond);
        for i = 1:12:numWaypoints
            show(gen3, qs(i,:)','PreservePlot',false); hold on;
            plot3(points(:,1),points(:,2),points(:,3),'-g','LineWidth',2);
            axis ([-0.5 0.6 -0.4 0.4 0 1]);
            view([45,0]);
            grid off;
            drawnow;
            waitfor(r);
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
            tic
            while true
                apiresponse = webread(url_from_HL);
                jsonresponse=jsondecode(apiresponse);
                if jsonresponse.status=="true"
                    currentvideoid = str2num(jsonresponse.id); %#ok<*ST2NM>  
                    previouslyknownid=currentvideoid;
                    break;
                elseif toc>3  
                    error("Not connected to the database");
                else
                    disp("Connecting to the database");
                    pause(1);
                end
            end
            break;
        end
        %% Running Kinova
        %Calculate joint velocity and acceleration at each waypoint with differentiation
        qs_deg = qs*180/pi;
        qs_deg_rev=flipud(qs)*180/pi;
        vel = diff(qs_deg)/dt;
        vel(1,:) = 0;
        vel(end+1,:) = 0; %#ok<SAGROW> 
        vel_rev = diff(qs_deg_rev)/dt;
        vel_rev(1,:)=0;
        vel_rev(end+1,:)=0; %#ok<SAGROW> 
        acc = diff(vel)/dt;
        acc(1,:) = 0;
        acc(end+1,:) = 0; %#ok<SAGROW> 
        acc_rev = diff(vel_rev)/dt;
        acc_rev(1,:) = 0;
        acc_rev(end+1,:) = 0; %#ok<SAGROW> 
        %Interpolate the joint position, velocity and acceleration to ensure the 0.001 seconds time step between two trajectory points
        timestamp = 0:0.001:t(end);
        qs_deg = interp1(t,qs_deg,timestamp);
        vel = interp1(t,vel,timestamp);
        acc = interp1(t,acc,timestamp);
        qs_deg_rev = interp1(t,qs_deg_rev,timestamp);
        vel_rev = interp1(t,vel_rev,timestamp);
        acc_rev = interp1(t,acc_rev,timestamp);
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
            webwrite(infor_to_db, jsonencode(struct('robo', 'n', 'motion', 'y', 'IK', 'n')));
            pause(5);
            webwrite(infor_to_db, jsonencode(struct('robo', 'n', 'motion', 'n', 'IK', 'n')));
            disp('SendJointAngles cmd error');
            tic
            while true
                apiresponse = webread(url_from_HL);
                jsonresponse=jsondecode(apiresponse);
                if jsonresponse.status=="true"
                    currentvideoid = str2num(jsonresponse.id); %#ok<*ST2NM>  
                    previouslyknownid=currentvideoid;
                    break;
                elseif toc>3  
                    error("Not connected to the database");
                else
                    disp("Connecting to the database");
                    pause(1);
                end
            end
            break;
        end
        pause(0.1)
        %Check if the robot has reached the starting position
        while 1
            [isOk,~, actuatorFb, ~] = gen3Kinova.SendRefreshFeedback();
            if isOk
                if max(abs(wrapTo360(qs_deg(1,:))-actuatorFb.position)) < 1
                    disp('Starting point reached.')
                    second_motion = [qs_deg.'; vel.'; acc.'; timestamp];
                    first_motion= [qs_deg_rev.'; vel_rev.'; acc_rev.'; timestamp];
                        auto_struct.first_motion= first_motion;
                        auto_struct.second_motion=second_motion;
                    break;
                end 
            else
                webwrite(infor_to_db, jsonencode(struct('robo', 'n', 'motion', 'y', 'IK', 'n')));
                pause(5);
                webwrite(infor_to_db, jsonencode(struct('robo', 'n', 'motion', 'n', 'IK', 'n')));
                % Updating the database id for new transform (main loop)
                tic
                while true
                    apiresponse = webread(url_from_HL);
                    jsonresponse=jsondecode(apiresponse);
                    if jsonresponse.status=="true"
                        currentvideoid = str2num(jsonresponse.id); %#ok<*ST2NM>  
                        previouslyknownid=currentvideoid;
                        break_main_loop= true;
                        break;
                    elseif toc>3  
                        error("Not connected to the database");
                    else
                        disp("Connecting to the database");
                        pause(1);
                    end
                end
                break;
            end
        end
        if break_main_loop
            break_main_loop=false;
            break;
        end
        %Send Pre-Computed Trajectory
        isOk = gen3Kinova.SendPreComputedTrajectory(qs_deg.', vel.', acc.', timestamp, size(timestamp,2));
        if isOk
            disp('success');
        else
            webwrite(infor_to_db, jsonencode(struct('robo', 'n', 'motion', 'y', 'IK', 'n')));
            pause(5);
            webwrite(infor_to_db, jsonencode(struct('robo', 'n', 'motion', 'n', 'IK', 'n')));
            disp('SendJointAngles cmd error');
            tic
            while true
                apiresponse = webread(url_from_HL);
                jsonresponse=jsondecode(apiresponse);
                if jsonresponse.status=="true"
                    currentvideoid = str2num(jsonresponse.id); %#ok<*ST2NM>  
                    previouslyknownid=currentvideoid;
                    break;
                elseif toc>3  
                    error("Not connected to the database");
                else
                    disp("Connecting to the database");
                    pause(1);
                end
            end
            break;
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
                webwrite(infor_to_db, jsonencode(struct('robo', 'n', 'motion', 'y', 'IK', 'n')));
                pause(5);
                webwrite(infor_to_db, jsonencode(struct('robo', 'n', 'motion', 'n', 'IK', 'n')));
                % Updating the database id for new transform (main loop)
                tic
                while true
                    apiresponse = webread(url_from_HL);
                    jsonresponse=jsondecode(apiresponse);
                    if jsonresponse.status=="true"
                        currentvideoid = str2num(jsonresponse.id); %#ok<*ST2NM>  
                        previouslyknownid=currentvideoid;
                        break_main_loop= true;
                        break;
                    elseif toc>3  
                        error("Not connected to the database");
                    else
                        disp("Connecting to the database");
                        pause(1);
                    end
                end
                break;
            end
        end
        if break_main_loop
            break_main_loop=false;
            break;
        end
        % Updating the database id for new transform (main loop)
        tic
        while true
            apiresponse = webread(url_from_HL);
            jsonresponse=jsondecode(apiresponse);
            if jsonresponse.status=="true"
                currentvideoid = str2num(jsonresponse.id); %#ok<*ST2NM>  
                previouslyknownid=currentvideoid;
                break;
            elseif toc>3  
                error("Not connected to the database");
            else
                disp("Connecting to the database");
                pause(1);
            end
        end
    end
    apiresponse = webread(url_from_HL);
    jsonresponse=jsondecode(apiresponse);
    if jsonresponse.status=="true"
        currentvideoid = str2num(jsonresponse.id); %#ok<*ST2NM> 
    end
    apiautomation = webread(automation);
    jsonautomation=jsondecode(apiautomation);
    if jsonautomation.status=="true"
        auto_bool=str2num(jsonautomation.auto);
        if auto_bool==2
            auto_bool=0;
            webwrite(auto_to_db,'autom',0);
            break
        end
    end
end    
if isempty(auto_struct.first_gripper_action) ||  isempty(auto_struct.second_gripper_action)...
            ||  isempty(auto_struct.second_motion) ||  isempty(auto_struct.first_motion)
    disp('cannot automate')
    return
end
while true 
    %% first robot motion
    close all
    f = figure;
    jointCmd = wrapTo360(auto_struct.first_motion(1:7, 1)');
    constraintType = int32(0); % no_constraint: 0; duration: 1; joint_velocity: 2;
    isOk = gen3Kinova.SendJointAngles(jointCmd, constraintType, speed, duration);
    if isOk
        disp('success');
    else
        webwrite(infor_to_db, jsonencode(struct('robo', 'n', 'motion', 'y', 'IK', 'n')));
        pause(5);
        webwrite(infor_to_db, jsonencode(struct('robo', 'n', 'motion', 'n', 'IK', 'n')));
        error('SendJointAngles cmd error');
    end
    pause(0.1)
    %Check if the robot has reached the starting position
    while 1
        [isOk,~, actuatorFb, ~] = gen3Kinova.SendRefreshFeedback();
        if isOk
            if max(abs(jointCmd-actuatorFb.position)) < 1
                disp('Starting point reached.')
                break;
            end 
        else
            webwrite(infor_to_db, jsonencode(struct('robo', 'n', 'motion', 'y', 'IK', 'n')));
            pause(5);
            webwrite(infor_to_db, jsonencode(struct('robo', 'n', 'motion', 'n', 'IK', 'n')));
            break_main_loop=true;
            break;
        end
    end
    if break_main_loop
        break_main_loop=false;
        break;
    end
    pause(0.1)
    isOk = gen3Kinova.SendPreComputedTrajectory(auto_struct.first_motion(1:7, :),auto_struct.first_motion(8:14, :),...
        auto_struct.first_motion(15:21, :) ,  auto_struct.first_motion(22, :),size(timestamp,2));
    if isOk
        disp('success');
    else
        webwrite(infor_to_db, jsonencode(struct('robo', 'n', 'motion', 'y', 'IK', 'n')));
        pause(5);
        webwrite(infor_to_db, jsonencode(struct('robo', 'n', 'motion', 'n', 'IK', 'n')));
        disp('SendJointAngles cmd error');
        tic
        break;
    end
    pause(0.1)
    while 1
        [isOk,~, actuatorFb, ~] = gen3Kinova.SendRefreshFeedback();
        if isOk
            if max(abs(wrapTo360(auto_struct.first_motion(1:7, end)')-actuatorFb.position)) < 1
                disp('End Point reached.')
                ind=ind+1;
                break;
            end 
        else
            webwrite(infor_to_db, jsonencode(struct('robo', 'n', 'motion', 'y', 'IK', 'n')));
            pause(5);
            webwrite(infor_to_db, jsonencode(struct('robo', 'n', 'motion', 'n', 'IK', 'n')));
            break_main_loop= true;
            break;
        end
    end
    if break_main_loop
        break_main_loop=false;
        break;
    end
    apiautomation = webread(automation);
    jsonautomation=jsondecode(apiautomation);
    if jsonautomation.status=="true"
        auto_bool=str2num(jsonautomation.auto);
    end
    if auto_bool==1
        webwrite(auto_to_db,'autom',0);
        break
    end
    pause(1)
    %% first gripper command
    toolCommand = int32(2);  
    toolCmd = auto_struct.first_gripper_action;
    toolDuration = 0;
    isOk = gen3Kinova.SendToolCommand(toolCommand, toolDuration, toolCmd);
    if isOk
        disp('Command sent to the gripper. Wait for the gripper to open.')
    else
        webwrite(infor_to_db, jsonencode(struct('robo', 'n', 'motion', 'y', 'IK', 'n')));
        pause(5);
        webwrite(infor_to_db, jsonencode(struct('robo', 'n', 'motion', 'n', 'IK', 'n')));
        disp('Command Error.');
    end
    apiautomation = webread(automation);
    jsonautomation=jsondecode(apiautomation);
    if jsonautomation.status=="true"
        auto_bool=str2num(jsonautomation.auto);
    end
    if auto_bool==1
        webwrite(auto_to_db,'autom',0);
        break
    end
    pause(1)
    %% second robot motion
    close all
    f = figure;
    jointCmd = wrapTo360(auto_struct.second_motion(1:7, 1)');
    constraintType = int32(0); % no_constraint: 0; duration: 1; joint_velocity: 2;
    isOk = gen3Kinova.SendJointAngles(jointCmd, constraintType, speed, duration);
    if isOk
        disp('success');
    else
        webwrite(infor_to_db, jsonencode(struct('robo', 'n', 'motion', 'y', 'IK', 'n')));
        pause(5);
        webwrite(infor_to_db, jsonencode(struct('robo', 'n', 'motion', 'n', 'IK', 'n')));
        error('SendJointAngles cmd error');
    end
    pause(0.1)
    %Check if the robot has reached the starting position
    while 1
        [isOk,~, actuatorFb, ~] = gen3Kinova.SendRefreshFeedback();
        if isOk
            if max(abs(jointCmd-actuatorFb.position)) < 1
                disp('Starting point reached.')
                break;
            end 
        else
            webwrite(infor_to_db, jsonencode(struct('robo', 'n', 'motion', 'y', 'IK', 'n')));
            pause(5);
            webwrite(infor_to_db, jsonencode(struct('robo', 'n', 'motion', 'n', 'IK', 'n')));
            break_main_loop=true;
            break;
        end
    end
    if break_main_loop
        break_main_loop=false;
        break;
    end
    pause(0.1)
    pause(0.1)
    isOk = gen3Kinova.SendPreComputedTrajectory(auto_struct.second_motion(1:7, :),auto_struct.second_motion(8:14, :),...
        auto_struct.second_motion(15:21, :) ,  auto_struct.second_motion(22, :),size(timestamp,2));    
    if isOk
        disp('success');
    else
        webwrite(infor_to_db, jsonencode(struct('robo', 'n', 'motion', 'y', 'IK', 'n')));
        pause(5);
        webwrite(infor_to_db, jsonencode(struct('robo', 'n', 'motion', 'n', 'IK', 'n')));
        disp('SendJointAngles cmd error');
        tic
        break;
    end
    pause(0.1)
    while 1
        [isOk,~, actuatorFb, ~] = gen3Kinova.SendRefreshFeedback();
        if isOk
            if max(abs(wrapTo360(auto_struct.second_motion(1:7, end)')-actuatorFb.position)) < 1
                disp('End Point reached.')
                ind=ind+1;
                break;
            end 
        else
            webwrite(infor_to_db, jsonencode(struct('robo', 'n', 'motion', 'y', 'IK', 'n')));
            pause(5);
            webwrite(infor_to_db, jsonencode(struct('robo', 'n', 'motion', 'n', 'IK', 'n')));
            break_main_loop= true;
            break;
        end
    end
    if break_main_loop
        break_main_loop=false;
        close all
        break;
    end
    apiautomation = webread(automation);
    jsonautomation=jsondecode(apiautomation);
    if jsonautomation.status=="true"
        auto_bool=str2num(jsonautomation.auto);
    end
    if auto_bool==1
        webwrite(auto_to_db,'autom',0);
        break
    end
    pause(1)
        %% second gripper command
    toolCommand = int32(2);    % Velocity control mode
    toolDuration = 0;
    toolCmd =  auto_struct.second_gripper_action; 
    isOk = gen3Kinova.SendToolCommand(toolCommand, toolDuration, toolCmd);
    if isOk
        disp('Command sent to the gripper. Wait for the gripper to close.')
    else
        webwrite(infor_to_db, jsonencode(struct('robo', 'n', 'motion', 'y', 'IK', 'n')));
        pause(5);
        webwrite(infor_to_db, jsonencode(struct('robo', 'n', 'motion', 'n', 'IK', 'n')));
        disp('Command Error.');
    end
    apiautomation = webread(automation);
    jsonautomation=jsondecode(apiautomation);
    if jsonautomation.status=="true"
        auto_bool=str2num(jsonautomation.auto);
    end
    if auto_bool==1
        webwrite(auto_to_db,'autom',0);
        break
    end
    pause(1)
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
%% Removes nearly stationary points from a path to streamline it, based on a specified epsilon threshold.
function [x_path, y_path, z_path] = cleanPath(smoothedPath, epsilon)
    x_path0 = smoothedPath(:,1)';
    y_path0 = smoothedPath(:,2)';
    z_path0 = smoothedPath(:,3)';
    dx = diff(x_path0);
    dy = diff(y_path0);
    dz = diff(z_path0);
    remove_indices = [false, (abs(dx) < epsilon) & (abs(dy) < epsilon) & (abs(dz) < epsilon)];
    x_path = x_path0(~remove_indices)';
    y_path = y_path0(~remove_indices)';
    z_path = z_path0(~remove_indices)';
    if x_path(end) ~= x_path0(end)
        x_path(end) = x_path0(end); y_path(end) = y_path0(end); z_path(end) = z_path0(end);
    end
end
%% Interpolates points along a path and normalizes the path's arc length, ensuring smooth and evenly distributed points.
function [xx, yy, zz] = interpolateAndNormalizePath(x_path, y_path, z_path)
    dt = 0.25;
    t = (0:dt:25)';
    theta = t * pi / (2 * t(end)); 
    tt = (1:length(x_path)) / length(x_path) * pi / 2;
    xx0 = interp1(tt, x_path, theta, 'cubic');
    yy0 = interp1(tt, y_path, theta, 'cubic');
    zz0 = interp1(tt, z_path, theta, 'cubic');  
    validIndices = ~isnan(xx0) & ~isnan(yy0) & ~isnan(zz0);
    xx0_clean = xx0(validIndices);
    yy0_clean = yy0(validIndices);
    zz0_clean = zz0(validIndices);
    dx = diff(xx0_clean);
    dy = diff(yy0_clean);
    dz = diff(zz0_clean);
    arcLen = cumsum(sqrt(dx.^2 + dy.^2 + dz.^2));
    arcLength = [0; arcLen];
    s = arcLength / arcLength(end);       
    s_uniform = linspace(0, 1, length(theta))';
    s_uniform = 0.5 - 0.5 * cos(pi * s_uniform);
    xx = interp1(s, xx0_clean, s_uniform, 'linear', 'extrap');
    yy = interp1(s, yy0_clean, s_uniform, 'linear', 'extrap');
    zz = interp1(s, zz0_clean, s_uniform, 'linear', 'extrap');
end
%% Checks if a given path is essentially a duplicate of any path in a collection of paths, considering a specified tolerance for similarity.
function isDuplicate = isPathDuplicate(currentPoints, distinctPaths, tolerance)
    isDuplicate = false;
    for i = 1:length(distinctPaths)
        existingPath = distinctPaths{i};
        
        % Ensure both paths have the same number of points for comparison
        % This step interpolates the current points to match the number in existingPath if necessary.
        % It's a simplistic approach; consider more sophisticated interpolation if needed.
        if size(currentPoints, 1) ~= size(existingPath, 1)
            currentPointsInterpolated = interp1(linspace(0, 1, size(currentPoints, 1)), currentPoints, linspace(0, 1, size(existingPath, 1)), 'linear', 'extrap');
        else
            currentPointsInterpolated = currentPoints;
        end
        
        % Compute Euclidean distances between corresponding points
        distances = sqrt(sum((currentPointsInterpolated - existingPath).^2, 2))/length(existingPath);
        
        % Check if average distance is below tolerance
        if mean(distances) < tolerance
            isDuplicate = true;
            break;
        end
    end
end

%% Plots quadrilaterals (obstacles) and distinct paths in 3D space, highlighting start and end points, to visually represent the path planning result.
function plotQuadrilaterals(startpoint, endpoint, quads, distinctPaths)
    figure;
    hold on;
    
    % Plot each quadrilateral
    for i = 1:length(quads)
        quad = quads{i}; % Access each quadrilateral
        fill3(quad(:,1), quad(:,2), quad(:,3), rand(1,3), 'FaceAlpha', 0.5); % Plot with random color
    end

    % Plot distinct paths
    colors = lines(numel(distinctPaths)); % Generate distinct colors for each path
    for i = 1:length(distinctPaths)
        path = distinctPaths{i}; % Access each path
        plot3(path(:,1), path(:,2), path(:,3), 'Color', colors(i,:), 'LineWidth', 2);
    end
    zlim([0,1])
    ylim([-0.45,0.95])
    xlim([0.3,1.2])

    % Plot start and end points
    scatter3(startpoint(1), startpoint(2), startpoint(3), 100, 'k', 'filled', 'DisplayName', 'Start Point');
    scatter3(endpoint(1), endpoint(2), endpoint(3), 100, 'k', 'filled', 'DisplayName', 'End Point');

    % Labeling
    text(startpoint(1), startpoint(2), startpoint(3), 'Start', 'HorizontalAlignment', 'right','VerticalAlignment','bottom');
    text(endpoint(1), endpoint(2), endpoint(3), 'End', 'HorizontalAlignment', 'right','VerticalAlignment','bottom');

    % Set axis labels and grid
    xlabel('X'); ylabel('Y'); zlabel('Z');
    grid on; axis equal; view(3);
    
    legend; % Add a legend to distinguish paths
    hold off;
end
