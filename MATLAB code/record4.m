clc
close all
clear

%% Setup
% Check available adaptors
info = imaqhwinfo;
disp('Available Adaptors:');
disp(info.InstalledAdaptors);

% Replace 'winvideo' with the first available adaptor if 'winvideo' is not available
adaptorName = 'winvideo';
if ~ismember(adaptorName, info.InstalledAdaptors)
    adaptorName = info.InstalledAdaptors{1};
    disp(['Using adaptor: ', adaptorName]);
end

% Define the video recording parameters
vid = videoinput(adaptorName, 1); % Adjust device ID as necessary
vid.FramesPerTrigger = 1; % Capture one frame at a time
vid.TriggerRepeat = Inf; % Continuously trigger
vid.ReturnedColorspace = 'rgb'; % Set the color space to RGB

% Set the recording duration
recordingDuration = 30; % Duration in seconds

% Create a unique file name based on the current timestamp
timestamp = datestr(now, 'yyyymmdd_HHMMSS');
videoFileName = ['recordedVideo_', timestamp, '.mp4'];
anglesFileName = ['jointAngles_', timestamp, '.txt'];

% Create a VideoWriter object to save the video
videoWriter = VideoWriter(videoFileName, 'MPEG-4');
videoWriter.FrameRate = 30; % Set frame rate to 30 fps
open(videoWriter);

% Open a file to save joint angles
anglesFile = fopen(anglesFileName, 'w');
if anglesFile == -1
    error('Failed to create joint angles file.');
end
fclose(anglesFile); % Close it immediately after creating to ensure it exists

% Start the video preview
preview(vid);

%% Initialize ROS
% Shutdown any existing ROS nodes in MATLAB
rosshutdown;

% Set environment variables
setenv('ROS_MASTER_URI', 'http://localhost:11311');
setenv('ROS_IP', '127.0.0.1');

% Initialize ROS node in MATLAB
rosinit('http://localhost:11311');

% Verify ROS connection
disp('ROS_MASTER_URI: ');
disp(getenv('ROS_MASTER_URI'));
disp('ROS_IP: ');
disp(getenv('ROS_IP'));

% List available topics to ensure connection
topicList = rostopic('list');
disp('Available ROS Topics:');
disp(topicList);

% Create the subscriber for joint states
global latestJointAngles
latestJointAngles = [];
jointStateSub = rossubscriber('/base_feedback/joint_state', 'sensor_msgs/JointState', @(src, msg) jointStateCallback(src, msg, anglesFileName));

%% Timer for Joint Angle Data Collection
% Create a timer to collect joint angle data at 5 Hz
angleTimer = timer('ExecutionMode', 'fixedRate', 'Period', 0.2, 'TasksToExecute', recordingDuration * 5, 'TimerFcn', @(~,~) collectAngleData(jointStateSub, anglesFileName));

try
    %% Start Video Acquisition and Angle Data Collection
    start(vid);
    start(angleTimer);

    % Start timing
    startTime = tic;
    disp('Recording started...');

    % Continuously acquire frames and save them
    while toc(startTime) < recordingDuration
        % Check if new frames are available
        if vid.FramesAvailable > 0
            % Get the acquired frame
            frame = getdata(vid, 1);
            
            % Overlay joint angles on the frame
            if ~isempty(latestJointAngles)
                angleText = sprintf('%.2f, ', latestJointAngles*180/pi);
                frame = insertText(frame, [10, 10], angleText, 'FontSize', 18, 'TextColor', 'white', 'BoxColor', 'black');
            end
            
            % Write the frame to the video file
            writeVideo(videoWriter, frame);
        end
    end

    % Stop the video acquisition
    stop(vid);

    % Stop the angle data collection timer
    stop(angleTimer);

    % Ensure both operations end simultaneously
    wait(angleTimer); % Wait for the timer to finish all tasks

    disp('Recording stopped.');

    % Close the VideoWriter object
    close(videoWriter);

    % Shutdown ROS
    rosshutdown;

    disp(['Video saved to ', fullfile(pwd, videoFileName)]);
    disp(['Joint angles saved to ', fullfile(pwd, anglesFileName)]);

catch ME
    % Cleanup in case of error
    stop(vid);
    delete(vid);
    if isvalid(angleTimer)
        stop(angleTimer);
        delete(angleTimer);
    end
    close(videoWriter);
    rosshutdown;
    rethrow(ME);
end

%% Cleanup
% Delete the video input object
delete(vid);

% Delete the angle timer
delete(angleTimer);

%% Callback function to handle joint state messages
function jointStateCallback(~, msg, anglesFileName)
    global latestJointAngles
    latestJointAngles = msg.Position;
    
    % Open the joint angles file in append mode
    anglesFile = fopen(anglesFileName, 'a');
    if anglesFile == -1
        disp('Failed to open joint angles file.'); % Debugging statement
        return;
    end
    
    % Write joint angles to file
    fprintf(anglesFile, '%.6f, ', latestJointAngles*180/pi);
    fprintf(anglesFile, '\n');
    
    % Close the file
    fclose(anglesFile);
end

%% Function to collect joint angle data
function collectAngleData(sub, anglesFileName)
    % Get the latest message from the subscriber
    msg = receive(sub, 1);
    % Call the joint state callback function with the received message
    jointStateCallback([], msg, anglesFileName);
end
