function [EPE, length] = calcEPE(stateEsti, stateTrue)
% Project:   Pentafocal-based Visual Inertial Odometry
% Function: calcEPE
%
% Description:
%   calculate End Point Error (EPE) divided by the total traveled trajectory
%
% Example:
%   OUTPUT :
%   EPE: percentage of end point error compared to length of dataset [%]
%   length: total traveled trajectory of a recording platform
%
%   INPUT :
%   stateEsti: current estimated 6-dof state w.r.t. ntimeRgb stamps
%   stateTrue: ground truth 6-dof state w.r.t. ntimeRgb stamps
%
% NOTE:
%
% Author: Pyojin Kim
% Email: pjinkim1215@gmail.com
% Website:
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% log:
% 2015-11-05 : Complete
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%

%% main part

% calculate end point error [m]
EPE_numerator = norm( stateEsti(1:3,end) - stateTrue(1:3,end) );


% calculate total traveled trajectory of a recording platform [m]
dDistance = diff(stateTrue(1:3,:),1,2);
EPE_denominator = sum(sqrt(sum(dDistance.^2,1)));

% result
length = EPE_denominator;
EPE = EPE_numerator / EPE_denominator;

end