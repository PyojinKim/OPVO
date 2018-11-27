function [meanRMD, RMD] = calcRMD(R_gc_esti, R_gc_true)

M = size(R_gc_true, 3);
rotationMatrixDifference = zeros(1, M);

% rotation matrix difference
for k = 1:M
    RgcTrue = R_gc_true(:,:,k);
    RgcEsti = R_gc_esti(:,:,k);
    
    rotationMatrixDifference(k) = acos((trace(RgcTrue.' * RgcEsti)-1)/2) * (180/pi);
end

% return error metric
rotationMatrixDifference = real(rotationMatrixDifference);
meanRMD = mean(rotationMatrixDifference);
RMD = rotationMatrixDifference;


end