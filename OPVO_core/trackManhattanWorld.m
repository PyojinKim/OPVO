function [R_cM_update, isTracked] = trackManhattanWorld(R_cM, surfaceNormalVector, optsMWO)

% pre-defined variables
iterNum = optsMWO.iterNum;
convergeAngle = optsMWO.convergeAngle;
halfApexAngle = optsMWO.halfApexAngle;
c = optsMWO.c;


%% track Manhattan world frame

% initial model parameters
R_cM_update = R_cM;
isTracked = 0;


% do mean shift iteration
denTemp = zeros(3,1) + 0.00001;
for iterCount = 1:iterNum
    
    R_cM = R_cM_update;
    numDirectionFound = 0;
    directionFound = [];
    
    % determine minimum number of normal vector
    numNormalVector = size(surfaceNormalVector, 2);
    numInCone = zeros(1, 3);
    for a = 1:3
        % projection on each axis (x, y, z)
        R_Mc = [R_cM(:,mod(a+3,3)+1), R_cM(:,mod(a+4,3)+1), R_cM(:,mod(a+5,3)+1)].';
        n_j = R_Mc * surfaceNormalVector;
        
        % check within half apex angle
        lambda = sqrt(n_j(1,:).*n_j(1,:) + n_j(2,:).*n_j(2,:));
        index = find(lambda <= sin(halfApexAngle));
        numInCone(a) = size(index, 2);
    end
    numInCone = sort(numInCone);
    
    minSampleNum = round(numNormalVector/20);
    if (numInCone(2) < minSampleNum)
        minSampleNum = (numInCone(1) + numInCone(2)) * 0.5;
    end
    
    % project to each Manhattan frame axis
    for a = 1:3
        R_Mc = [R_cM(:,mod(a+3,3)+1), R_cM(:,mod(a+4,3)+1), R_cM(:,mod(a+5,3)+1)].';
        n_j = R_Mc * surfaceNormalVector;
        
        lambda = sqrt(n_j(1,:).*n_j(1,:) + n_j(2,:).*n_j(2,:));
        index = find(lambda < sin(halfApexAngle));
        n_j_inlier = n_j(:,index);
        tan_alfa = lambda(index)./abs(n_j(3,index));
        alfa = asin(lambda(index));
        m_j = [alfa./tan_alfa.*n_j_inlier(1,:)./n_j_inlier(3,:);
            alfa./tan_alfa.*n_j_inlier(2,:)./n_j_inlier(3,:)];
        
        select = ~isnan(m_j);
        select2 = select(1,:).*select(2,:);
        select3 = find(select2 == 1);
        m_j = m_j(:,select3);
        
        if (size(m_j, 2) >= minSampleNum)
            
            % compute mean shift
            [s_j, denTemp(a,1)] = MeanShift(m_j, c);
            
            % compute the Ma
            alfa = norm(s_j);
            ma_p = tan(alfa)/alfa * s_j;
            R_cM_update(:,a) = R_Mc.' * [ma_p; 1];
            R_cM_update(:,a) = R_cM_update(:,a) / norm(R_cM_update(:,a));
            numDirectionFound = numDirectionFound + 1;
            directionFound = [directionFound a];
        end
    end
    
    % handle numDirectionFound is not three
    if (numDirectionFound < 2)
        R_cM_update = R_cM;
        isTracked = 0;
        return;
    elseif (numDirectionFound == 2)
        v1 = R_cM_update(:,directionFound(1));
        v2 = R_cM_update(:,directionFound(2));
        v3 = cross(v1,v2);
        R_cM_update(:,6-(directionFound(1)+directionFound(2))) = v3;
        if (abs(det(R_cM_update)+1) < 0.5)
            R_cM_update(:,6-(directionFound(1)+directionFound(2))) = -v3;
        end
    end
    
    % maintain orthogonality on SO(3)
    [U,~,V] = svd([R_cM_update(:,1)*denTemp(1,1), R_cM_update(:,2)*denTemp(2,1), R_cM_update(:,3) * denTemp(3,1)]);
    R_cM_update = U * V';
    
    % check convergence
    if (acos((trace(R_cM.' * R_cM_update) - 1)/2) < convergeAngle)
        break;
    end
end

isTracked = 1;


end


