function [x_moving_path, y_moving_path, z_moving_path] = moving_path(constr, startpoint, endpoint, xx, yy, zz, modified_path)
% Split the string into individual coordinate strings


coordStrings = strsplit(modified_path, ') ');
coordStrings = coordStrings (1:end-1); % Remove the last empty element
% Initialize coordinate vectors
x_m_p = zeros(length(coordStrings),1);
y_m_p = zeros(length(coordStrings),1);
z_m_p = zeros(length(coordStrings),1);

% Parse each coordinate string
for i = 1:length(coordStrings)
    coordString = coordStrings{i};
    coordString = strrep(coordString, '(', '');
    coordString = strrep(coordString, ')', '');
    coords = str2num(coordString); % Convert string to numbers
    x_m_p(i) = coords(3);
    y_m_p(i) = -coords(1);
    z_m_p(i) = coords(2);
end


eps=0.001;
% Detecting changes
changedX = xx - x_m_p > eps;
changedY = yy - y_m_p > eps;
changedZ = zz - z_m_p > eps;
changedPoints = changedX | changedY | changedZ;
% Extracting changed points
x_changed = x_m_p(changedPoints);
y_changed = y_m_p(changedPoints);
z_changed = z_m_p(changedPoints);
% Display or save the changed points
disp([x_changed; y_changed; z_changed]);
numPoints = size(x_changed, 2); % Number of points/columns in the array
if mod(numPoints, 2) == 0
    middleIndex = numPoints / 2; % Choose the latter of the two middle points for even number of points
else
    middleIndex = (numPoints + 1) / 2; % Direct middle for odd number of points
end
middlePoint = [x_changed(middleIndex); y_changed(middleIndex); z_changed(middleIndex)]';

% initialize the first rrt
rrt = RrtPlanner(25,3,1,constr);
rrt.SetStart(startpoint);
rrt.SetGoal(middlePoint);
rrt.SetUpDataStructures();
rrt.Run()
delete(rrt);
% obstacle
obstacleFilename = 'constr.txt'; 
% final rrt
seedsPerAxis = 7;
treesMax = seedsPerAxis^3*3+2;
rrt = RrtPlanner(treesMax,seedsPerAxis,obstacleFilename,constr);
rrt.drawingSkipsPerDrawing = 30;
rrt.SetStart(startpoint);
rrt.SetGoal(middlePoint);
rrt.SetUpDataStructures();
rrt.Run()
x_path01=rrt.smoothedPath(:,1)';
y_path01=rrt.smoothedPath(:,2)';
z_path01=rrt.smoothedPath(:,3)';
epsilon = 0.05; % Set a small epsilon value
% initialize the second rrt
rrt = RrtPlanner(25,3,1,constr);
rrt.SetStart(middlePoint);
rrt.SetGoal(endpoint);
rrt.SetUpDataStructures();
rrt.Run()
delete(rrt);
% obstacle
obstacleFilename = 'constr.txt'; 
% final rrt
seedsPerAxis = 7;
treesMax = seedsPerAxis^3*3+2;
rrt = RrtPlanner(treesMax,seedsPerAxis,obstacleFilename,constr);
rrt.drawingSkipsPerDrawing = 30;
rrt.SetStart(middlePoint);
rrt.SetGoal(endpoint);
rrt.SetUpDataStructures();
rrt.Run()
x_path02=rrt.smoothedPath(:,1)';
y_path02=rrt.smoothedPath(:,2)';
z_path02=rrt.smoothedPath(:,3)';
% first+second rrt
x_path0=[x_path01,x_path02];
y_path0=[y_path01,y_path02];
z_path0=[z_path01,z_path02];
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
x_moving_path = interp1(s, xx0_clean, s_uniform, 'linear', 'extrap');
y_moving_path = interp1(s, yy0_clean, s_uniform, 'linear', 'extrap');
z_moving_path = interp1(s, zz0_clean, s_uniform, 'linear', 'extrap');

end