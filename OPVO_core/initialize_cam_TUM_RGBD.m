function [cam] = initialize_cam_TUM_RGBD(TUMRGBDdataset, maxPyramidLevel)
% camera calibration parameters from TUM RGBD dataset


% intrinsic camera calibration parameters
cam.K = TUMRGBDdataset.rgb.K;

cam.k1 = TUMRGBDdataset.rgb.distortion(1);
cam.k2 = TUMRGBDdataset.rgb.distortion(2);
cam.p1 = TUMRGBDdataset.rgb.distortion(3);
cam.p2 = TUMRGBDdataset.rgb.distortion(4);
cam.p3 = TUMRGBDdataset.rgb.distortion(5);


% intrinsic camera calibration K parameters for multi-level pyramid
K_pyramid = zeros(3,3,maxPyramidLevel);
K_pyramid(:,:,1) = cam.K;

% perform pyramid reduction with average values
for k = 2:maxPyramidLevel
    K_pyramid(:,:,k) = downsampleKmatrix(K_pyramid(:,:,k-1));
end
cam.K_pyramid = K_pyramid;


% image size parameters
cam.nRows = 480;
cam.nCols = 640;


% scale factor to recover depth image
cam.scaleFactor = TUMRGBDdataset.depth.scaleFactor;


end