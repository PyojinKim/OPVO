function [rawICLNUIMdataset] = rawICLNUIMdataset_load(datasetPath)
% Project:   Patch-based Illumination invariant Visual Odometry (PIVO)
% Function: rawICLNUIMdataset_load
%
% Description:
%   get raw rgb, depth, and Vicon data from ICL NUIM dataset
%
% Example:
%   OUTPUT:
%   rawICLNUIMdataset:
%
%   INPUT:
%   datasetPath: directory of folder which includes the groundtruth.txt from this m.file
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
% 2017-02-10: Complete
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%


%% import data from text files


% import ground-truth data
delimiter = ' ';
textData_groundtruth = importdata([datasetPath '/groundtruth.txt'], delimiter);


% import time_sync (rgb and depth) data
fileID = fopen([datasetPath, '/associations.txt'], 'r');
textData_timesync = textscan(fileID, '%f64 %s %f64 %s');
fclose(fileID);


%% load ground-truth and rgb data


% load ground-truth data
rawICLNUIMdataset.vicon.time = textData_groundtruth(:,1).';
rawICLNUIMdataset.vicon.p_gc = [textData_groundtruth(:,2).'; -textData_groundtruth(:,3).'; textData_groundtruth(:,4).'];
rawICLNUIMdataset.vicon.q_gc = [textData_groundtruth(:,8).'; textData_groundtruth(:,5).'; textData_groundtruth(:,6).'; textData_groundtruth(:,7).'];


% convert camera frame (to my convention)
q_gc_temp = rawICLNUIMdataset.vicon.q_gc;
numData = size(q_gc_temp, 2);
q_gc_convert = zeros(4, numData);
for k = 1:numData
    R_gc_true = q2r(q_gc_temp(:, k));
    angle_true = rotmtx2angle(R_gc_true.');
    
    angle_true_convert = [-angle_true(1); angle_true(2); -angle_true(3)];
    R_gc_true_convert = angle2rotmtx(angle_true_convert).';
    
    q_gc_convert(:,k) = r2q(R_gc_true_convert);
end
rawICLNUIMdataset.vicon.q_gc = q_gc_convert;


% load synchronized rgb data
K = eye(3);
K(1,1) = 481.2;  % fx
K(2,2) = 480.0;  % fy
K(1,3) = 319.5;  % cx
K(2,3) = 239.5;  % cy
distortion = [0, 0, 0, 0, 0];  % d0 d1 d2 d3 d4


rawICLNUIMdataset.rgb.time = textData_timesync{3}.';
rawICLNUIMdataset.rgb.imgName = textData_timesync{4}.';
rawICLNUIMdataset.rgb.K = K;
rawICLNUIMdataset.rgb.distortion = distortion;


% load synchronized depth data
rawICLNUIMdataset.depth.time = textData_timesync{1}.';
rawICLNUIMdataset.depth.imgName = textData_timesync{2}.';
rawICLNUIMdataset.depth.scaleFactor = 5000;


end
