clc;
close all;
clear variables; %clear classes;
rand('state',0); % rand('state',sum(100*clock));
dbstop if error;


%% basic setup for OPVO

% choose the experiment case
% ICL NUIM dataset (1~8)
expCase = 1;

% are figures drawn?
% 1 : yes, draw figures to see current status
% 0 : no, just run OPVO
toVisualize = 1;

% are data results saved?
% 1 : yes, save the variables and results
% 0 : no, just run OPVO
toSave = 1;


setupParams_ICL_NUIM;


% load ICL NUIM dataset data
rawICLNUIMdataset = rawICLNUIMdataset_load(datasetPath);


% camera calibration parameters
[ICLNUIMdataset] = getSyncTUMRGBDdataset(rawICLNUIMdataset, imInit, M);
optsMWO = load_param_MWO;
cam = initialize_cam_TUM_RGBD(ICLNUIMdataset, optsMWO.imagePyramidLevel);


%% load ground truth data

% ground truth trajectory in ICL NUIM dataset
R_gc_true = zeros(3,3,M);
p_gc_true = zeros(3,M);
T_gc_true = cell(1,M);
for k = 1:M
    % camera body frame
    R_gc_true(:,:,k) = q2r(ICLNUIMdataset.vicon.q_gc_Sync(:,k));
    p_gc_true(:,k) = ICLNUIMdataset.vicon.p_gc_Sync(:,k);
    T_gc_true{k} = [ R_gc_true(:,:,k), p_gc_true(:,k);
        zeros(1,3),           1; ];
end
if (toVisualize)
    figure; hold on; axis equal;
    L = 0.1; % coordinate axis length
    A = [0 0 0 1; L 0 0 1; 0 0 0 1; 0 L 0 1; 0 0 0 1; 0 0 L 1]';
    
    for k = 1:10:M
        T = T_gc_true{k};
        B = T * A;
        plot3(B(1,1:2),B(2,1:2),B(3,1:2),'-r','LineWidth',1); % x: red
        plot3(B(1,3:4),B(2,3:4),B(3,3:4),'-g','LineWidth',1); % y: green
        plot3(B(1,5:6),B(2,5:6),B(3,5:6),'-b','LineWidth',1); % z: blue
    end
    plot3(p_gc_true(1,:),p_gc_true(2,:),p_gc_true(3,:),'k','LineWidth',2);
    
    title('ground truth trajectory of cam0 frame')
    xlabel('x'); ylabel('y'); zlabel('z');
end


% generate ground truth trajectory in vector form
stateTrue = zeros(6,M);
stateTrue(1:3,:) = p_gc_true;
for k = 1:size(p_gc_true,2)
    [yaw, pitch, roll] = dcm2angle(R_gc_true(:,:,k));
    stateTrue(4:6,k) = [roll; pitch; yaw];
end


%% main OPVO part

% 1. feature tracking pre-defined variables for DEMO
systemInited_ft = false;

imageHeight = 480;
imageWidth = 640;
imagePixelNum = imageHeight * imageWidth;
imgSize = [imageWidth imageHeight];

imageCur = uint8( zeros(imgSize(2), imgSize(1)) );
imageLast = uint8( zeros(imgSize(2), imgSize(1)) );

showCount = 0;
showSkipNum = 2;
showDSRate = 2;
showSize = [imageWidth/showDSRate imageHeight/showDSRate];

imageShow = uint8( zeros(showSize(2), showSize(1)) );
harrisLast = uint8( zeros(showSize(2), showSize(1)) );

maxFeatureNumPerSubregion = 2;
xSubregionNum = 12;
ySubregionNum = 8;
totalSubregionNum = xSubregionNum * ySubregionNum;
MAXFEATURENUM = maxFeatureNumPerSubregion * totalSubregionNum;

xBoundary = 20;
yBoundary = 20;
subregionWidth = (imageWidth - 2 * xBoundary) / xSubregionNum;
subregionHeight = (imageHeight - 2 * yBoundary) / ySubregionNum;

maxTrackDis = 100;
winSize = 15;

featuresCur = cell(1, 2 * MAXFEATURENUM);
featuresLast = cell(1, 2 * MAXFEATURENUM);
featuresFound = zeros(1, 2 * MAXFEATURENUM);
featuresError = zeros(1, 2 * MAXFEATURENUM);

featuresIndFromStart = 0;
featuresInd = zeros(1, 2 * MAXFEATURENUM);

totalFeatureNum = 0;
subregionFeatureNum = zeros(1, 2 * totalSubregionNum);

imagePointsCur = cell(0);
imagePointsLast = cell(0);


% 2. Manhattan frame tracking for MWO
systemInited_MF = false;

R_gc1 = R_gc_true(:,:,1);
R_gc_OPVO = zeros(3,3,M);
R_gc_OPVO(:,:,1) = R_gc1;


% 3. frame to frame motion estimation pre-defined variables for DEMO
systemInited_VO = false;

T_gc_current = T_gc_true{1};
T_gc_OPVO = cell(1, M);
T_gc_OPVO{1} = T_gc_current;


% 4. make figures to visualize current status
if (toVisualize)
    % create figure
    h = figure(10);
    set(h,'Color',[1 1 1]);
    set(h,'Units','pixels','Position',[150 400 1700 600]);
    ha1 = axes('Position',[0.025,0.1 , 0.3,0.8]);
    axis off;
    ha2 = axes('Position',[0.350,0.1 , 0.3,0.8]);
    axis off;
    ha3 = axes('Position',[0.675,0.1 , 0.3,0.8]);
    axis equal; grid on; hold on;
end


% do OPVO
for imgIdx = 1:M
    %% 1. feature tracking : re-assign variables for next ieration
    
    % image
    imageLast = imageCur;
    imageCur = getImgInTUMRGBDdataset(datasetPath, ICLNUIMdataset, cam, imgIdx, 'gray');
    
    imageShow = imresize(imageLast, (1/showDSRate));
    harrisLast = cv.cornerHarris(imageShow, 'BlockSize', 3);
    
    % features
    featuresTemp = featuresLast;
    featuresLast = featuresCur;
    featuresCur = featuresTemp;
    
    % image points
    imagePointsTemp = imagePointsLast;
    imagePointsLast = imagePointsCur;
    imagePointsCur = imagePointsTemp;
    imagePointsCur = cell(0);
    
    % for the first time in this loop
    if (~systemInited_ft)
        systemInited_ft = true;
    elseif (systemInited_ft)
        %% 1. feature tracking : feature detection with goodFeaturesToTrack
        
        recordFeatureNum = totalFeatureNum;
        for i = 1:ySubregionNum
            for j = 1:xSubregionNum
                ind = xSubregionNum * (i-1) + j;
                numToFind = maxFeatureNumPerSubregion - subregionFeatureNum(ind);
                
                if (numToFind>0)
                    
                    subregionLeft = xBoundary + (subregionWidth * (j-1)) + 1;
                    subregionTop = yBoundary + (subregionHeight * (i-1)) + 1;
                    
                    uStart = subregionLeft;
                    uEnd = subregionLeft + subregionWidth - 1;
                    vStart = subregionTop;
                    vEnd = subregionTop + subregionHeight - 1;
                    subregionTemp = imageLast(vStart:vEnd, uStart:uEnd);
                    
                    featuresTemp = cv.goodFeaturesToTrack(subregionTemp, 'QualityLevel', 0.1, 'MinDistance', 5.0, 'BlockSize', 3, 'UseHarrisDetector', true, 'K', 0.04);
                    for k=1:size(featuresTemp, 2)
                        featuresLast(totalFeatureNum+k) = featuresTemp(k);
                    end
                    
                    numFound = 0;
                    for k=1:size(featuresTemp, 2)
                        featuresLast{totalFeatureNum+k}(1) = featuresLast{totalFeatureNum+k}(1) + (subregionLeft-1);
                        featuresLast{totalFeatureNum+k}(2) = featuresLast{totalFeatureNum+k}(2) + (subregionTop-1);
                        
                        xInd = round( (featuresLast{totalFeatureNum+k}(1) + 0.5) / showDSRate );
                        yInd = round( (featuresLast{totalFeatureNum+k}(2) + 0.5) / showDSRate );
                        
                        if (harrisLast(yInd, xInd) >= 1e-6)
                            numFound = numFound + 1;
                            featuresIndFromStart = featuresIndFromStart +1;
                            
                            featuresLast{totalFeatureNum+numFound}(1) = featuresLast{totalFeatureNum+k}(1);
                            featuresLast{totalFeatureNum+numFound}(2) = featuresLast{totalFeatureNum+k}(2);
                            featuresInd(totalFeatureNum+numFound) = featuresIndFromStart;
                        end
                    end
                    
                    totalFeatureNum = totalFeatureNum + numFound;
                    subregionFeatureNum(ind) = subregionFeatureNum(ind) + numFound;
                end
            end
        end
        
        
        %% 1. feature tracking : feature tracking with KLT tracker
        
        [featuresCur, featuresFound, featuresError]  = cv.calcOpticalFlowPyrLK(imageLast, imageCur, featuresLast(1:totalFeatureNum), 'WinSize', [winSize winSize], 'MaxLevel', 3);
        
        for i=1:totalSubregionNum
            subregionFeatureNum(i) = 0;
        end
        
        featureCount = 0;
        for i=1:totalFeatureNum
            trackDis = sqrt( (featuresLast{i}(1) - featuresCur{i}(1))*(featuresLast{i}(1) - featuresCur{i}(1)) + (featuresLast{i}(2) - featuresCur{i}(2))*(featuresLast{i}(2) - featuresCur{i}(2)) );
            
            if (~(trackDis > maxTrackDis || featuresCur{i}(1) < xBoundary || featuresCur{i}(1) > (imageWidth - xBoundary) ...
                    || featuresCur{i}(2) < yBoundary || featuresCur{i}(2) > (imageHeight - yBoundary)))
                
                xInd = floor( (featuresLast{i}(1) - xBoundary) / subregionWidth ) + 1;
                yInd = floor( (featuresLast{i}(2) - yBoundary) / subregionHeight ) + 1;
                ind = xSubregionNum * (yInd-1) + xInd;
                
                if (subregionFeatureNum(ind) < maxFeatureNumPerSubregion)
                    featureCount = featureCount + 1;
                    subregionFeatureNum(ind) = subregionFeatureNum(ind) + 1;
                    
                    featuresCur{featureCount}(1) = featuresCur{i}(1);
                    featuresCur{featureCount}(2) = featuresCur{i}(2);
                    featuresLast{featureCount}(1) = featuresLast{i}(1);
                    featuresLast{featureCount}(2) = featuresLast{i}(2);
                    featuresInd(featureCount) = featuresInd(i);
                    
                    point.u = (featuresCur{featureCount}(1) - cam.K(1,3)) / cam.K(1,1);
                    point.v = (featuresCur{featureCount}(2) - cam.K(2,3)) / cam.K(2,2);
                    point.ind = featuresInd(featureCount);
                    imagePointsCur{end+1} = point;
                    
                    if (i > recordFeatureNum)
                        point.u = (featuresLast{featureCount}(1) - cam.K(1,3)) / cam.K(1,1);
                        point.v = (featuresLast{featureCount}(2) - cam.K(2,3)) / cam.K(2,2);
                        imagePointsLast{end+1} = point;
                    end
                end
            end
        end
        totalFeatureNum = featureCount;
    end
    
    
    %% 2. Manhattan frame tracking
    
    % image
    imageCurForMW = getImgInTUMRGBDdataset(datasetPath, ICLNUIMdataset, cam, imgIdx, 'rgb');
    depthCurForMW = getImgInTUMRGBDdataset(datasetPath, ICLNUIMdataset, cam, imgIdx, 'depth');
    [imageCurForMW, depthCurForMW] = getImgPyramid(imageCurForMW, depthCurForMW, optsMWO.imagePyramidLevel);
    
    % surface normal vector
    [sNV, sPP] = estimateSurfaceNormalGradient(imageCurForMW, depthCurForMW, cam, optsMWO);
    
    
    % for the first time in this loop
    if (~systemInited_MF)
        
        % initialize and seek the dominant MF
        [MF_can, FindMF] = seekManhattanWorld(sNV, optsMWO);
        if (FindMF == 1)
            R_cM = ClusterMMF(MF_can, optsMWO.ratio);
            if (isempty(R_cM) == 0)
                systemInited_MF = true;
                disp('Initialization done!');
            end
            R_cM = R_cM{1};
            
            % for result convertion
            R_c1M = R_cM;
        end
    elseif (systemInited_MF)
        
        % tracking MF
        [R_cM, IsTracked] = trackManhattanWorld(R_cM, sNV, optsMWO);
        
        % if lost tracking
        if (IsTracked == 0)
            disp(num2str(imgIdx));
            disp('lost tracking!');
            break;
        end
        
        % update current camera rotation
        R_gc_current = R_gc1 * R_c1M * inv(R_cM);
        R_gc_OPVO(:,:,imgIdx) = R_gc_current;
    end
    
    
    %% 3. frame to frame motion estimation : re-assign variables for next ieration
    
    % for the first time in this loop
    if (~systemInited_VO)
        initTime = ICLNUIMdataset.depth.ntime(imgIdx);
        systemInited_VO = true;
    elseif  (systemInited_VO)
        %% 3. frame to frame motion estimation : find relationship between points
        
        depthImageTemp = getImgInTUMRGBDdataset(datasetPath, ICLNUIMdataset, cam, imgIdx-1, 'depth');
        imagePointsLastNum = size(imagePointsLast, 2);
        imagePointsCurNum = size(imagePointsCur, 2);
        
        ipRelations = cell(0);
        for i=1:imagePointsLastNum
            ipFound = false;
            for j=1:imagePointsCurNum
                if (imagePointsCur{j}.ind == imagePointsLast{i}.ind)
                    ipFound = true;
                end
                if (imagePointsCur{j}.ind >= imagePointsLast{i}.ind)
                    break;
                end
            end
            
            if (ipFound)
                ipr.x = imagePointsLast{i}.u;
                ipr.y = imagePointsLast{i}.v;
                ipr.z = imagePointsCur{j}.u;
                ipr.h = imagePointsCur{j}.v;
                
                clear uTemp vTemp
                uTemp = cam.K(1,1) * ipr.x + cam.K(1,3);
                vTemp = cam.K(2,2) * ipr.y + cam.K(2,3);
                depthTemp = depthImageTemp( round(vTemp), round(uTemp));
                
                if (depthTemp > 0.3 && depthTemp < 7)
                    ipr.s = depthTemp;
                    ipr.v = 1;
                else
                    ipr.s = 0;
                    ipr.v = 0;
                end
                
                ipRelations{end+1} = ipr;
            end
        end
        
        
        %% 3. frame to frame motion estimation : estimate 6 DoF motion
        
        % initialize 6 DoF model parameters
        tx = 0;
        ty = 0;
        tz = 0;
        phi = 0;
        theta = 0;
        psi = 0;
        
        
        % estimate the rotational motion [R]
        R_21 = inv(R_gc_OPVO(:,:,imgIdx)) * R_gc_OPVO(:,:,imgIdx-1);
        
        tempEulerAngle = rotmtx2angle(R_21);
        phi = tempEulerAngle(1);
        theta = tempEulerAngle(2);
        psi = tempEulerAngle(3);
        
        
        % estimate the translational motion [t]
        iterNum = 150;
        ipRelationsNum = size(ipRelations, 2);
        ptNumWithDepthRec = 0;
        meanValueWithDepthRec = 100000;
        for iterCount = 1:iterNum
            
            ipRelations2 = cell(0);
            ipy2 = cell(0);
            
            ptNumWithDepth = 0;
            meanValueWithDepth = 0;
            
            for i = 1:ipRelationsNum
                clear ipr
                ipr = ipRelations{i};
                
                u1 = ipr.x;
                v1 = ipr.y;
                u2 = ipr.z;
                v2 = ipr.h;
                
                if (abs(ipr.v - 1) < 0.5 || abs(ipr.v - 2) < 0.5) % with depth point
                    
                    X_1 = u1 * ipr.s;
                    Y_1 = v1 * ipr.s;
                    Z_1 = ipr.s;
                    jacobian1Temp = df_depth_1_dt([tx; ty; tz; phi; theta; psi; u2; v2; X_1; Y_1; Z_1; u1; v1]);
                    jacobian2Temp = df_depth_2_dt([tx; ty; tz; phi; theta; psi; u2; v2; X_1; Y_1; Z_1; u1; v1]);
                    
                    clear ipr3 ipr4
                    ipr3.x = jacobian1Temp(1);
                    ipr3.y = jacobian1Temp(2);
                    ipr3.z = jacobian1Temp(3);
                    
                    ipr4.x = jacobian2Temp(1);
                    ipr4.y = jacobian2Temp(2);
                    ipr4.z = jacobian2Temp(3);
                    
                    clear y3 y4
                    y3 = f_depth_1([tx; ty; tz; phi; theta; psi; u2; v2; X_1; Y_1; Z_1; u1; v1]);
                    y4 = f_depth_2([tx; ty; tz; phi; theta; psi; u2; v2; X_1; Y_1; Z_1; u1; v1]);
                    
                    
                    if (ptNumWithDepthRec < 50 || iterCount < 25 || sqrt(y3 * y3 + y4 * y4) < 2 * meanValueWithDepthRec)
                        ipRelations2{end+1} = ipr3;
                        ipy2{end+1} = y3;
                        
                        ipRelations2{end+1} = ipr4;
                        ipy2{end+1} = y4;
                        
                        ptNumWithDepth = ptNumWithDepth + 1;
                        meanValueWithDepth = meanValueWithDepth + sqrt(y3 * y3 + y4 * y4);
                    else
                        ipRelations{i}.v = -1;
                    end
                end
            end
            
            meanValueWithDepth = meanValueWithDepth / (ptNumWithDepth + 0.01);
            ptNumWithDepthRec = ptNumWithDepth;
            meanValueWithDepthRec = meanValueWithDepth;
            
            ipRelations2Num = size(ipRelations2, 2);
            if (ipRelations2Num > 10)
                
                matA = zeros(ipRelations2Num, 3);
                matB = zeros(ipRelations2Num, 1);
                
                for i=1:ipRelations2Num
                    ipr2 = ipRelations2{i};
                    
                    matA(i, 1) = ipr2.x;
                    matA(i, 2) = ipr2.y;
                    matA(i, 3) = ipr2.z;
                    matB(i, 1) = - 0.1 * ipy2{i};
                end
                
                matAt = matA.';
                matAtA = matAt * matA;
                matAtB = matAt * matB;
                matX = cv.solve(matAtA, matAtB);   % deltaTemp = inv(matA.' * matA) * matA.' * matB;
                
                tx = tx + matX(1, 1);
                ty = ty + matX(2, 1);
                tz = tz + matX(3, 1);
                
                deltaT = sqrt(matX(1, 1) * 100 * matX(1, 1) * 100 +...
                    matX(2, 1) * 100 * matX(2, 1) * 100 +...
                    matX(3, 1) * 100 * matX(3, 1) * 100);
                
                if (deltaT < 0.0001)
                    break;
                end
            end
        end
    end
    
    
    %% 4. update 6 DoF camera pose and visualization
    
    if (imgIdx >= 2)
        % update current camera pose
        t_21 = [tx; ty; tz];
        R_21 = angle2rotmtx([phi; theta; psi]);
        T_21 = [R_21, t_21;
            0, 0, 0, 1];
        
        T_gc_current = T_gc_current * inv(T_21);
        T_gc_OPVO{imgIdx} = T_gc_current;
        
        
        % visualize current status
        plots_ICL_NUIM;
    end
    
    
end

% convert camera pose representation
stateEsti_OPVO = zeros(6, M);
R_gc_OPVO = zeros(3,3,M);
for k = 1:M
    R_gc_OPVO(:,:,k) = T_gc_OPVO{k}(1:3,1:3);
    stateEsti_OPVO(1:3,k) = T_gc_OPVO{k}(1:3,4);
    [yaw, pitch, roll] = dcm2angle(R_gc_OPVO(:,:,k));
    stateEsti_OPVO(4:6,k) = [roll; pitch; yaw];
end


%% plot error metric value (RPE, ATE)

% 1) OPVO motion estimation trajectory results
figure;
plot3(stateTrue(1,:),stateTrue(2,:),stateTrue(3,:),'k','LineWidth',2); hold on; grid on;
plot3(stateEsti_OPVO(1,:),stateEsti_OPVO(2,:),stateEsti_OPVO(3,:),'r','LineWidth',2);
legend('True','OPVO Matlab'); plot_inertial_frame(0.5); axis equal; view(-158, 38);
xlabel('x [m]','fontsize',10); ylabel('y [m]','fontsize',10); zlabel('z [m]','fontsize',10); hold off;


% 2) calculate RPE / ATE / EPE error metric for OPVO
[RPE_RMSE_OPVO,RPE_OPVO] = calcRPE(T_gc_OPVO, T_gc_true, 30, 'RMSE');
[ATE_RMSE_OPVO,ATE_OPVO] = calcATE(T_gc_OPVO, T_gc_true, 'RMSE');
[EPE_OPVO,lengthDataset] = calcEPE(stateEsti_OPVO, stateTrue);
fprintf('*******************************\n')
fprintf('RMSE of RPE [drift m/s] : %f \n' , RPE_RMSE_OPVO);
fprintf('RMSE of ATE [m] : %f \n' , ATE_RMSE_OPVO);
fprintf('EPE [%%] : %f \n' , EPE_OPVO);
fprintf('Total traveled distance [m] : %f \n' , lengthDataset);
fprintf('*******************************\n')


% 3) draw figures for RPE, ATE
figure;
plot(RPE_OPVO); grid on;
xlabel('Time [ # of images]','fontsize',10); ylabel('OPVO drift error [m/s]','fontsize',10);
title('Relative Pose Error (RPE) versus time index','fontsize',15)

figure;
plot(ATE_OPVO); grid on;
xlabel('Time [ # of images]','fontsize',10); ylabel('OPVO absolute trajectory error [m]','fontsize',10);
title('Absolute Trajectory Error (ATE) versus time index','fontsize',15)


% 4) calculate rotation matrix difference
[RMD_MEAN_OPVO, RMD_OPVO] = calcRMD(R_gc_OPVO, R_gc_true);
fprintf('MEAN of RMD [deg] : %f \n' , RMD_MEAN_OPVO);
fprintf('std. of RMD [deg] : %f \n' , std(RMD_OPVO));


% 5) draw figures for rotation matrix difference
figure;
plot(RMD_OPVO, 'r*'); hold on; grid on;
curve_x = 1:M;
p = polyfit(curve_x, RMD_OPVO, 3);
curve_y = polyval(p, curve_x); curve_y(curve_y < 0) = 0;
plot(curve_x, curve_y, 'g-', 'LineWidth',5); hold off; axis tight; ylabel('RMD (deg)');
legend('RMD', 'Fitting Curve','Location','northwest');

figure;
ploterrhist(RMD_OPVO, 'bins', 25);


%% save the experiment data for BMVC 2017

if (toSave)
    save([SaveDir '/OPVO.mat']);
end

