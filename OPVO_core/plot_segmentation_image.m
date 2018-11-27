function plot_segmentation_image(R_cM, surfaceNormalVector, surfacePixelPoint, imageShow, optsMWO)

% assign parameters
imageHeight = size(imageShow, 1);
imageWidth = size(imageShow, 2);
halfApexAngle = optsMWO.halfApexAngle;


%% project normal vectors to each Manhattan frame axis

numNormalVector = size(surfaceNormalVector, 2);
surfaceAxisIndex = ones(1, numNormalVector) * -1000;
for a = 1:3
    % projection on each axis (x, y, z)
    R_Mc = [R_cM(:,mod(a+3,3)+1), R_cM(:,mod(a+4,3)+1), R_cM(:,mod(a+5,3)+1)].';
    n_j = R_Mc * surfaceNormalVector;
    
    % check within half apex angle
    lambda = sqrt(n_j(1,:).*n_j(1,:) + n_j(2,:).*n_j(2,:));
    index = find(lambda <= sin(halfApexAngle));
    surfaceAxisIndex(:, index) = a;
end


%% plot plane segmentation results

% RGB mask (x-y-z)
rgbMask = uint8(zeros(imageHeight, imageWidth, 3));
for k = 1:numNormalVector
    
    tempPixel = surfacePixelPoint(:,k);
    a = surfaceAxisIndex(k);
    
    if (a == 1)
        % red
        for m = 1:size(tempPixel, 2)
            u = tempPixel(1, m);
            v = tempPixel(2, m);
            rgbMask(v, u, 1) = 200;
            rgbMask(v, u, 2) = 0;
            rgbMask(v, u, 3) = 0;
        end
    elseif (a == 2)
        % green
        for m = 1:size(tempPixel, 2)
            u = tempPixel(1, m);
            v = tempPixel(2, m);
            rgbMask(v, u, 1) = 0;
            rgbMask(v, u, 2) = 200;
            rgbMask(v, u, 3) = 0;
        end
    elseif (a == 3)
        % blue
        for m = 1:size(tempPixel, 2)
            u = tempPixel(1, m);
            v = tempPixel(2, m);
            rgbMask(v, u, 1) = 0;
            rgbMask(v, u, 2) = 0;
            rgbMask(v, u, 3) = 200;
        end
    else
        % gray
        for m = 1:size(tempPixel, 2)
            u = tempPixel(1, m);
            v = tempPixel(2, m);
            rgbMask(v, u, 1) = 100;
            rgbMask(v, u, 2) = 100;
            rgbMask(v, u, 3) = 100;
        end
    end
end


% plot plane segmentation results
imshow(imageShow, []); hold on;
h_rgbMask = imshow(rgbMask, []);
set(h_rgbMask, 'AlphaData', 0.5);


end