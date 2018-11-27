function [TUMRGBDdatasetNet] = getSyncTUMRGBDdataset(rawTUMRGBDdataset, imInit, M)
% Project:    Depth enhanced visual odometry
% Function:  getSyncTUMRGBDdataset
%
% Description:
%   get synchronized rgb, depth and Vicon data from rawTUMRGBDdataset
%
% Example:
%   OUTPUT:
%   TUMRGBDdataset: compact and synchronized rgb, depth, Vicon datasets
%
%   INPUT:
%   rawTUMRGBDdataset: raw TUM RGBD dataset variable from 'rawTUMRGBDdataset_load(datasetPath)'
%   imInit: first image index (1-based index)
%   M: number of images
%
%
% NOTE:
%
% Author: Pyojin Kim
% Email: pjinkim1215@gmail.com
% Website:
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% log:
% 2017-01-22: Complete
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%


% copy from rawTUMRGBDdataset
TUMRGBDdataset = rawTUMRGBDdataset;


% normalize the time stamps in rgb, depth, ground truth data
minReferenceTime = min([TUMRGBDdataset.rgb.time(1), TUMRGBDdataset.depth.time(1), TUMRGBDdataset.vicon.time(1)]);
TUMRGBDdataset.rgb.ntime = double(TUMRGBDdataset.rgb.time - minReferenceTime);
TUMRGBDdataset.depth.ntime = double(TUMRGBDdataset.depth.time - minReferenceTime);
TUMRGBDdataset.vicon.ntime = double(TUMRGBDdataset.vicon.time  - minReferenceTime);


%% synchronize depth, vicon ntime w.r.t. rgb ntime


% depth part
matchingTimeIdx_depth = zeros(size(TUMRGBDdataset.rgb.ntime,2),2);
for i = 1:size(matchingTimeIdx_depth,1)
    temp_difference = abs( TUMRGBDdataset.depth.ntime - TUMRGBDdataset.rgb.ntime(i) );
    [~,minTimeIdx] = min(temp_difference);
    
    matchingTimeIdx_depth(i,1) = minTimeIdx; % ntime in depth index
    matchingTimeIdx_depth(i,2) = i;               % ntime in rgb index
end

TUMRGBDdataset.rgb.matchingTimeIdx_depth = matchingTimeIdx_depth.';


% vicon part
matchingTimeIdx_Vicon = zeros(size(TUMRGBDdataset.rgb.ntime,2),2);
for i = 1:size(matchingTimeIdx_Vicon,1)
    temp_difference = abs( TUMRGBDdataset.vicon.ntime - TUMRGBDdataset.rgb.ntime(i) );
    [~,minTimeIdx] = min(temp_difference);
    
    matchingTimeIdx_Vicon(i,1) = minTimeIdx;  % ntime in Vicon index
    matchingTimeIdx_Vicon(i,2) = i;                % ntime in rgb index
end

TUMRGBDdataset.vicon.ntime_Sync  = TUMRGBDdataset.vicon.ntime(matchingTimeIdx_Vicon(:,1));
TUMRGBDdataset.vicon.p_gc_Sync = TUMRGBDdataset.vicon.p_gc(:,matchingTimeIdx_Vicon(:,1));
TUMRGBDdataset.vicon.q_gc_Sync = TUMRGBDdataset.vicon.q_gc(:,matchingTimeIdx_Vicon(:,1));


%% extract pre-defined interval from [imInit] to [imInit+M-1]


% re-load rgb information
TUMRGBDdatasetNet.rgb.ntime = TUMRGBDdataset.rgb.ntime(:,(imInit):(imInit+M-1));
for k=1:M
    TUMRGBDdatasetNet.rgb.imgName{k} = TUMRGBDdataset.rgb.imgName{imInit+k-1};
end
TUMRGBDdatasetNet.rgb.K = TUMRGBDdataset.rgb.K;
TUMRGBDdatasetNet.rgb.distortion = TUMRGBDdataset.rgb.distortion;


% re-load depth information
for k=1:M
    depthIdx = TUMRGBDdataset.rgb.matchingTimeIdx_depth(1,imInit+k-1);
    
    TUMRGBDdatasetNet.depth.ntime(k) = TUMRGBDdataset.depth.ntime(:,depthIdx);
    TUMRGBDdatasetNet.depth.imgName{k} = TUMRGBDdataset.depth.imgName{depthIdx};
end
TUMRGBDdatasetNet.depth.scaleFactor = TUMRGBDdataset.depth.scaleFactor;


% re-load ground-truth data
TUMRGBDdatasetNet.vicon.ntime_Sync = TUMRGBDdataset.vicon.ntime_Sync(:,(imInit):(imInit+M-1));
TUMRGBDdatasetNet.vicon.p_gc_Sync = TUMRGBDdataset.vicon.p_gc_Sync(:,(imInit):(imInit+M-1));
TUMRGBDdatasetNet.vicon.q_gc_Sync = TUMRGBDdataset.vicon.q_gc_Sync(:,(imInit):(imInit+M-1));


end




