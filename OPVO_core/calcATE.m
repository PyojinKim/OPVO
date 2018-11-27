function [errorResult_ATE, trajectoryNormError] = calcATE(T_gc_esti, T_gc_true, errorType)
% Project:   Patch-based Illumination invariant Visual Odometry (PIVO)
% Function: calcATE
%
% Description:
%   calculate Absolute Trajectory Error (ATE) defined well in TUM benchmark dataset.
%   It is designed to compare the absolute distances btwn the estimated and the ground truth trajectory
%
% Example:
%   OUTPUT :
%   errorResult_ATE: error result of ATE with respect to the 'errorType'
%   trajectoryNormError: norm error of the translational components in trajectory
%
%   INPUT :
%   T_gc_esti: current estimated SE(3)
%   T_gc_true: ground truth SE(3)
%   errorType : the type of error related to ATE ( RMSE / MEAN / MEDIAN )
%
% NOTE:
%     Absolute Trajectory Error (ATE) is well explained in the below paper
%     "A Benchmark for the Evaluation of RGB-D SLAM Systems" at page 579
%
% Author: Pyojin Kim
% Email: pjinkim1215@gmail.com
% Website:
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% log:
% 2017-04-05 : Complete
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%

%% set parameters

numImg = size(T_gc_esti, 2);

trajectoryError = cell(1,numImg);
trajectoryNormError = zeros(1,numImg);


%% main ATE part

for k = 1:numImg
    % calculate ATE
    estiPart = T_gc_esti{k};
    truePart = T_gc_true{k};
    trajectoryError{k} =   inv( truePart ) * ( estiPart );
    trajectoryNormError(k) = norm( trajectoryError{k}(1:3,4) );
end


%% assigne the error value
if ( strcmp(errorType,'RMSE') )
    
    errorResult_ATE = sqrt( sum( trajectoryNormError.^2 ) / numImg );
    
elseif ( strcmp(errorType,'MEAN') )
    
    errorResult_ATE = mean( trajectoryNormError );
    
elseif ( strcmp(errorType,'MEDIAN') )
    
    errorResult_ATE = median( trajectoryNormError );
    
else
    
    error('errorType error!!! Try again!!');
    
end

end
