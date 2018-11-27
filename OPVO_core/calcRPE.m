function [errorResult_RPE, trajectoryNormError] = calcRPE(T_gc_esti, T_gc_true, timeInterval, errorType)
% Project:   Patch-based Illumination invariant Visual Odometry (PIVO)
% Function: calcRPE
%
% Description:
%   calculate Relative Pose Error (RPE) defined well in TUM benchmark dataset.
%   It is designed to measure the local accuracy of the trajectory over a fixed time interval
%
% Example:
%   OUTPUT :
%   errorResult_RPE: error result of RPE with respect to the 'errorType'
%   trajectoryNormError: norm error of the translational components in trajectory
%
%   INPUT :
%   T_gc_esti: current estimated SE(3)
%   T_gc_true: ground truth SE(3)
%   timeInterval : a fixed time interval ( 30 if 30Hz -> drift m/s )
%   errorType : the type of error related to RPE ( RMSE / MEAN / MEDIAN )
%
% NOTE:
%     Relative Pose Error (RPE) is well explained in the below paper
%     "A Benchmark for the Evaluation of RGB-D SLAM Systems" at page 578
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
numRPE = numImg - timeInterval;

trajectoryError = cell(1,numRPE);
trajectoryNormError = zeros(1,numRPE);


%% main RPE part

for k = 1:numRPE
    % truePart
    Qtrue = T_gc_true{k};
    QtrueNext = T_gc_true{k + timeInterval};
    
    % estiPart
    Pesti = T_gc_esti{k};
    PestiNext = T_gc_esti{k + timeInterval};
    
    % calculate RPE
    truePart = inv(Qtrue) * QtrueNext;
    estiPart = inv(Pesti) * PestiNext;
    trajectoryError{k} =   inv( truePart ) * ( estiPart );
    trajectoryNormError(k) = norm( trajectoryError{k}(1:3,4) );
end


%% assigne the error value
if ( strcmp(errorType,'RMSE') )
    
    errorResult_RPE = sqrt( sum( trajectoryNormError.^2 ) / numRPE );
    
elseif ( strcmp(errorType,'MEAN') )
    
    errorResult_RPE = mean( trajectoryNormError );
    
elseif ( strcmp(errorType,'MEDIAN') )
    
    errorResult_RPE = median( trajectoryNormError );
    
else
    
    error('errorType error!!! Try again!!');
    
end

end
