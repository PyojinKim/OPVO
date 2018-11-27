function plot_sphere_compass(R_cM, surfaceNormalVector, optsMWO)

% assign parameters
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


%% plot sphere compass results

xAxisIndex = (surfaceAxisIndex == 1);
yAxisIndex = (surfaceAxisIndex == 2);
zAxisIndex = (surfaceAxisIndex == 3);
otherIndex = (surfaceAxisIndex == -1000);

xAxisNV = surfaceNormalVector(:, xAxisIndex);
yAxisNV = surfaceNormalVector(:, yAxisIndex);
zAxisNV = surfaceNormalVector(:, zAxisIndex);
otherNV = surfaceNormalVector(:, otherIndex);


% plot sphere compass results with normal vector points
plot_unit_sphere(18, 0.5); hold on; grid on; axis equal;
plot3(xAxisNV(1,:), xAxisNV(2,:), xAxisNV(3,:), 'r.');
plot3(yAxisNV(1,:), yAxisNV(2,:), yAxisNV(3,:), 'g.');
plot3(zAxisNV(1,:), zAxisNV(2,:), zAxisNV(3,:), 'b.');
plot3(otherNV(1,:), otherNV(2,:), otherNV(3,:), 'k.');
plot_body_frame(R_cM, 3); hold off;
%R_cM_view = R_cM * angle2rotmtx([pi/2;0;0]).'; % rotate 90 degree along x-axis


end