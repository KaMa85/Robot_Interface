clc
close all
clear
 %% Read and Play the Recorded Video File
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
vid.FramesPerTrigger = Inf; % Record until manually stopped
vid.ReturnedColorspace = 'rgb'; % Set the color space to RGB

% Set the recording duration
recordingDuration = 10; % Duration in seconds

% Create a unique file name based on the current timestamp
fileName = ['recordedVideo_', datestr(now, 'yyyymmdd_HHMMSS'), '.mp4'];

% Create a VideoWriter object to save the video
videoWriter = VideoWriter(fileName, 'MPEG-4');
open(videoWriter);

% Start the video preview
preview(vid);

% Start acquiring frames
start(vid);

disp('Recording started...');

% Record for the specified duration
pause(recordingDuration);

% Stop the video acquisition
stop(vid);
disp('Recording stopped.');

% Retrieve the acquired frames
frames = getdata(vid);

% Write the frames to the video file
for i = 1:size(frames, 4)
    writeVideo(videoWriter, frames(:,:,:,i));
end

% Close the VideoWriter object
close(videoWriter);

% Clean up
delete(vid);
clear vid;

disp(['Video saved to ', fullfile(pwd, fileName)]);

%% Record Joints' Angles
% Step 1: Shutdown any existing ROS nodes in MATLAB
rosshutdown;

% Step 2: Set environment variables
setenv('ROS_MASTER_URI', 'http://localhost:11311');
setenv('ROS_IP', '127.0.0.1');

% Step 3: Initialize ROS node in MATLAB
rosinit('http://localhost:11311');

% Step 4: Verify ROS connection
disp('ROS_MASTER_URI: ');
disp(getenv('ROS_MASTER_URI'));
disp('ROS_IP: ');
disp(getenv('ROS_IP'));

% Step 5: List available topics to ensure connection
topicList = rostopic('list');
disp('Available ROS Topics:');
disp(topicList);

% Step 6: Create the subscriber for joint states
jointStateSub = rossubscriber('/base_feedback/joint_state', 'sensor_msgs/JointState');

% Step 7: Set the timer
pauseDuration = 0.5; % Duration between updates in seconds

% Step 8: Run the loop to get joint angles
disp('Waiting for joint state messages...');
while true
    % Receive the latest joint state message
    jointStateMsg = receive(jointStateSub, 10); % Wait up to 10 seconds
    % Display joint angles
    disp('Joint Angles:');
    disp(jointStateMsg.Position);
    % Pause for the specified duration
    pause(pauseDuration);
end

% Note: To stop the script, you will need to interrupt it manually (Ctrl+C in the Command Window).
