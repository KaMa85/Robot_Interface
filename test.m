
clc; close all; clear;
%%
iterations = 2;
levels = 7;
model = 'affine';
option = 1;


videodirectory="C:\xampp\htdocs\";
imagedirectory="C:\xampp\htdocs\postprocessed_images\";
databaseip="http://localhost";
currentvideoid=0;
previouslyknownid=0;

varstoclear={'apiresponse','imagelocation','index', 'jsonpoints','jsonresponse','objectFrame','objectRegion',...
        'pointImage','points','points_out_re','response','tracker','url','videoFileReader','videoPlayer'};
while true
    %%%%%connection to the data base Ali,
    while previouslyknownid>=currentvideoid
        url = databaseip+'/selectnewest.php';
        apiresponse = webread(url);
        jsonresponse=jsondecode(apiresponse);
        if jsonresponse.status=="true"
            currentvideoid = str2num(jsonresponse.video_id);
        end
        pause(0.1);
    end
    previouslyknownid=currentvideoid;
    disp(videodirectory+jsonresponse.location);
tic
%% Create System objects for reading and displaying video and for drawing 
% a bounding box of the object.
filename = videodirectory+jsonresponse.location; %%'crop_DJI_0001.avi';
hVideoSrc = VideoReader(filename);
videoFileReader = vision.VideoFileReader(filename);
videoPlayer = vision.VideoPlayer;
framerate = round(hVideoSrc.FrameRate);
width = hVideoSrc.Width;
height = hVideoSrc.Height;
border = 10;
x1 = (border/100)*width;
y1 = (border/100)*height;
x2 = width - (2*border/100)*width;
y2 = height - (2*border/100)*height;

%% Read the first video frame, which contains the object, define the region.
a1a = 3; %%%%% seconds to skip
read(hVideoSrc,framerate*a1a);
%objectFrame = step(videoFileReader);
objectFrame = readFrame(hVideoSrc);
objectRegion = [x1, y1, x2, y2];


%% As an alternative, you can use the following commands to select the 
% objectImage = insertShape(objectFrame, 'Rectangle', objectRegion,'Color', 'red', 'LineWidth',5);
% figure; imshow(objectImage); 

%% Detect interest points in the object region.
np = 500;
point = detectMinEigenFeatures(rgb2gray(objectFrame), 'ROI', objectRegion);
point = point.selectStrongest(np);

%% Display the detected points.
pointImage = insertMarker(objectFrame, point, '+', 'Color', 'red','size',5);
figure, imshow(pointImage), title('Detected interest points');
pointImage = insertMarker(objectFrame, point, '+', 'Color', 'green','size',5);
points = point;
% 
% %% Create a tracker object.
tracker = vision.PointTracker('MaxBidirectionalError', 1, 'NumPyramidLevels', 5);

%% Initialize the tracker.
% initialize(tracker, points, objectFrame);
initialize(tracker, points.Location, objectFrame);


% %% Create output video
% outputVideo = VideoWriter('tracker_short_20210909_005_132107-1.avi');
% outputVideo.FrameRate = hVideoSrc.FrameRate;
% open(outputVideo)
% writeVideo(outputVideo,pointImage);

%% Read, track, display points, and results in each video frame.
np = points.Count;
index = 1;
%read(hVideoSrc,1);
val = zeros(np,hVideoSrc.NumFrames - ((framerate*a1a)+1));
while hasFrame(hVideoSrc)
      frame = readFrame(hVideoSrc);
      [points, validity] = step(tracker, frame);
      val(:,index) = validity;
      out = insertMarker(frame, points(validity, :), '+');
      step(videoPlayer, out);
%       writeVideo(outputVideo,out);
      location_points_top (:,:,index) = points;
      index = index + 1;
end
% close(outputVideo)
val_zero = all(val ~= 0, 2);
valid_pts = find(val_zero ~= 0);
location_points_top = location_points_top(valid_pts, :,:);
np = size(location_points_top,1);
%% Release the video reader and player.
% release(videoPlayer);
% release(videoFileReader);

%% get movement information of feature points
size_lp = size(location_points_top);

for i = 1:size_lp(1)
    for j = 2:size_lp(3)
        X(i,j-1) = location_points_top(i,1,j) - location_points_top(i,1,1);
        Y(i,j-1) = location_points_top(i,2,j) - location_points_top(i,2,1);
        D(i,j-1) = (X(i,j-1)^2+Y(i,j-1)^2)^0.5;        
        theta(i,j-1) = atan(Y(i,j-1)/X(i,j-1));
    end
end
sq = 35;
threshold = [0:0.2:2]; %%%%%%%%%%%AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA%%%%%%%%%%%
aa = zeros(np,length(threshold));
bb = zeros(np,length(threshold));
count = 1;
    for i = 1:size_lp(1) % i for center point of the LCR
        for j = i+1:size_lp(1) % j for each point 
           if ((location_points_top(i,1,1)  - location_points_top(j,1,1))^2 + (location_points_top(i,2,1)  - location_points_top(j,2,1))^2)^0.5  < sq
            LCR_d = ((location_points_top(i,1,:)  - location_points_top(j,1,:)).^2 + (location_points_top(i,2,:)  - location_points_top(j,2,:)).^2).^0.5;
            LCR_d = detrend(squeeze(LCR_d),3);
                   for kk = 1: length(threshold)

                        if std(LCR_d) > threshold(kk) 
                            aa(i,kk) = 1;
                            bb(j,kk) = 1;
                         
                        end

                   end
           end

        end
    end
   %Create empty cell array to store DB IDs of threshold results
    interestPointsDbIDs = {};
    % plot threshold in the image
    for ii = 1 : length(threshold)
        J2 = imadjust(objectFrame,[0; 1],[0.7; 1]);
        ali=[location_points_top(find(aa(:,ii)),:,1); location_points_top(find(bb(:,ii)),:,1)];
        pointImage = insertMarker(J2, ali, '+', 'Color', 'red','size',5);
        %figure;  imshow(pointImage); 
        title(['threshold = ',num2str(threshold(ii)), threshold(ii)]);
        %%add result to database
        jsonpoints = jsonencode(ali);
        ali1 = size(ali);
        rows = ali1(:,1);
        numPoints = num2str(rows);
        %This webresult uploads the interstpoints and returns the DB ID of
        %the uploaded points
        webResponse = webwrite(databaseip+'/InsertPoints.php','threshold',threshold(ii),'points',jsonpoints,'numPoints',numPoints);
        disp(webResponse);
        webResponseInt = str2double(webResponse);
        interestPointsDbIDs{end+1} = webResponseInt;
        %saveas(gcf,strcat(num2str(ii),'.png'));
    end


    % plot threshold in the image
    for ii = 1 : length(threshold)
%       J2 = imadjust(objectFrame,[0; 1],[0.7; 1]);
        ali=[location_points_top(find(aa(:,ii)),:,1); location_points_top(find(bb(:,ii)),:,1)];
        ali1 = size(ali);
        rows = ali1(:,1);
        blankimage=zeros(1278,2272);
        pointImage = insertMarker(blankimage,ali, '*', 'Color', 'red','Size',14); %%%
        figure('Name','Kansas Result'), imshow(pointImage), text(1500,1200,['Number of Points = ',num2str(rows)]...
        ,'Color','green','FontSize',25);
    end


%Threshhold versus number of points plot
% output=ones(ii,1);
%     for ii = 1:length(threshold)
%         ali=[location_points_top(find(aa(:,ii)),:,1); location_points_top(find(bb(:,ii)),:,1)];
%         ali1=size(ali);
%         rowsp = ali1(:,1);
%         oot=output.*threshold';
%         oot2=output.*rowsp';
%     end
    

%%%Make alpha channel for result
pointImage = pointImage/max(pointImage(:));
A=blankimage;
A(pointImage(:,:,1)==1)=1;


%upload photo and data to database
    jsonPointsIDArray=jsonencode(interestPointsDbIDs);
    disp(jsonPointsIDArray);
    imagelocation=imagedirectory+jsonresponse.video_name+".png";
    imwrite(pointImage,imagelocation,'Alpha', A);
    response = webwrite(databaseip+'/insert.php','prevideo_id',jsonresponse.video_id,'transformName',jsonresponse.transformName,'json_points',jsonPointsIDArray,'image_location',jsonresponse.video_name+".png");


    disp(response);

    
    clear(varstoclear{:});
end
toc