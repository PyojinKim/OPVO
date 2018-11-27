function [surfaceNormalVector, surfacePixelPoint, labelMask] = estimateSurfaceNormalGrid(depthCur, cam, optsMWO)

% assign current image pyramid
L = optsMWO.imagePyramidLevel;
K = cam.K_pyramid(:,:,L);
cellsize = optsMWO.cellsize;


%% depth image smoothing

% remove extreme depth values
rawDepthCur = depthCur;
rawDepthCur(rawDepthCur < 0.5 | rawDepthCur > 5) = -1000;
rawDepthCur(rawDepthCur == -1000) = 0;

% guided filter (not use)
%rawImageCur = imageCur;
%filterDepthCur = imguidedfilter(rawDepthCur, rawImageCur);
%filterDepthCur(filterDepthCur < 0.5 | filterDepthCur > 5) = -1000;
%filterDepthCur(filterDepthCur == -1000) = 0;

% final filtered depth map
depthMapFiltered = rawDepthCur;


%% square uniform grid image segmentation

% create the cell
depthMap = depthMapFiltered;
imageHeight = size(depthMap, 1);
imageWidth = size(depthMap, 2);
cellNumX = floor(imageWidth/cellsize);
cellNumY = floor(imageHeight/cellsize);
numCell = cellNumX * cellNumY;

% uniform grid labeling
labelMask = zeros(imageHeight, imageWidth);
cellCount = 0;
for y = 1:cellNumY
    for x = 1:cellNumX
        
        % u and v index
        uStart = (x-1)*cellsize + 1;
        uEnd = (x-1)*cellsize + cellsize;
        vStart = (y-1)*cellsize + 1;
        vEnd = (y-1)*cellsize + cellsize;
        
        % assign label
        labelMask(vStart:vEnd, uStart:uEnd) = cellCount;
        cellCount = cellCount + 1;
    end
end
labelMask = uint32(labelMask);


%% normal vectors and pixel points of each cell

% recover 3D coordinates
vertexMap = zeros(imageHeight, imageWidth, 3);
[x,y] = meshgrid(1:imageWidth, 1:imageHeight);
vertexMap(:,:,1) = (x-K(1,3)).*depthMap/K(1,1);
vertexMap(:,:,2) = (y-K(2,3)).*depthMap/K(2,2);
vertexMap(:,:,3) = depthMap;


% find plane for each cell
surfaceNormalVector = zeros(3, numCell);
surfacePixelPoint = cell(1, numCell);
cellCount = 0;
for y = 1:cellNumY
    for x = 1:cellNumX
        
        % u and v index
        uStart = (x-1)*cellsize + 1;
        uEnd = (x-1)*cellsize + cellsize;
        vStart = (y-1)*cellsize + 1;
        vEnd = (y-1)*cellsize + cellsize;
        
        % 3D point cloud
        X = vertexMap(vStart:vEnd,uStart:uEnd, 1);
        Y = vertexMap(vStart:vEnd,uStart:uEnd, 2);
        Z = vertexMap(vStart:vEnd,uStart:uEnd, 3);
        X = reshape(X, [cellsize*cellsize 1]);
        Y = reshape(Y, [cellsize*cellsize 1]);
        Z = reshape(Z, [cellsize*cellsize 1]);
        
        % 2D pixel point
        [u, v] = meshgrid(uStart:uEnd, vStart:vEnd);
        uPixel = reshape(u, [cellsize*cellsize 1]).';
        vPixel = reshape(v, [cellsize*cellsize 1]).';
        
        % find the good 3-D points for fitting
        Z_median = median(Z);
        index = find(abs((Z - Z_median)/Z_median) < 0.05);
        Z = Z(index);
        X = X(index);
        Y = Y(index);
        
        % do the fitting
        if (numel(index) >= (0.5*cellsize^2))
            cellCount = cellCount + 1;
            
            surfaceNormalVector(:, cellCount) = fitPlane(X,Y,Z);
            surfacePixelPoint{cellCount} = [uPixel; vPixel];
        end
    end
end
surfaceNormalVector(:, (cellCount+1):end) = [];
surfacePixelPoint((cellCount+1):end) = [];


% normalize surface normal vectors
numNormalVector = size(surfaceNormalVector, 2);
for k = 1:numNormalVector
    surfaceNormalVector(:,k) = surfaceNormalVector(:,k) / norm(surfaceNormalVector(:,k));
end


end



% plane fitting
function nv = fitPlane(X,Y,Z)
% fit the plane
A = [X,Y,Z];
b = -ones(numel(X),1);
nv = A\b;
nv = nv / norm(nv);
end



