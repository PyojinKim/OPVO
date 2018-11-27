function [optsMWO] = load_param_MWO
% Project:   Patch-based illumination-variant DVO
% Function: load_param_MWO
%
% Description:
%   get the initial parameters with respect to the algorithm
%
% Example:
%   OUTPUT:
%   optsMWO: options for MWO process like below
%
%   INPUT:
%
%
% NOTE:
%     The parameters below are initialized as the Zhou paper
%
% Author: Pyojin Kim
% Email: pjinkim1215@gmail.com
% Website:
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% log:
% 2017-02-25: Complete
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%


% RGB and depth image parameters
optsMWO.imagePyramidLevel = 2;
optsMWO.mainCamICSL = 2;


% Manhattan world frame seeking parameters
optsMWO.numInitialization = 200;
optsMWO.iterNum = 200;
optsMWO.convergeAngle = 0.0001 * (pi/180);
optsMWO.halfApexAngle = 15 * (pi/180);
optsMWO.c = 20;
optsMWO.ratio = 0.1;


% surface normal vector
optsMWO.cellsize = 10;
optsMWO.regionSize = 10;
optsMWO.regularizer = 0.1;


end






