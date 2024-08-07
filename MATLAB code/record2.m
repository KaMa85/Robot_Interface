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
jointStateSub = rossubscriber('/base_feedback/joint_state', 'sensor_msgs/JointState', @(src, msg) jointStateCallback(src, msg, anglesFileName));

%% Start Video Acquisition
start(vid);

%% Record video and joint angles
pauseDuration = 0.2; % Duration between angle data updates in seconds
frameRate = 30; % Video frame rate
framesPerUpdate = frameRate * pauseDuration; % Frames to capture per update
startTime = tic; % Start the timer

disp('Recording started...');
while toc(startTime) < recordingDuration
    % Collect frames continuously
    frames = getdata(vid, framesPerUpdate, 'numeric');
    % Write frames to the video file
    for i = 1:size(frames, 4)
        writeVideo(videoWriter, frames(:, :, :, i));
    end
    
    % Pause for the specified duration for joint angle updates
    pause(pauseDuration);
end

%% Cleanup
% Stop the video acquisition
stop(vid);
disp('Recording stopped.');

% Close the VideoWriter object
close(videoWriter);

% Shutdown ROS
rosshutdown;

disp(['Video saved to ', fullfile(pwd, videoFileName)]);
disp(['Joint angles saved to ', fullfile(pwd, anglesFileName)]);

%% Callback function to handle joint state messages
function jointStateCallback(~, msg, anglesFileName)
    disp('Joint state message received'); % Debugging statement
    disp(msg.Position); % Debugging statement to display the received positions
    
    % Open the joint angles file in append mode
    anglesFile = fopen(anglesFileName, 'a');
    if anglesFile == -1
        disp('Failed to open joint angles file.'); % Debugging statement
        return;
    end
    
    % Write joint angles to file
    fprintf(anglesFile, '%.6f, ', msg.Position*180/pi);
    fprintf(anglesFile, '\n');
    
    % Close the file
    fclose(anglesFile);
end
