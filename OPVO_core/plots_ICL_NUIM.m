if (toVisualize)
    %% prerequisite to visualize
    
    p_gc_OPVO = zeros(3, imgIdx);
    for k = 1:imgIdx
        p_gc_OPVO(:,k) = T_gc_OPVO{k}(1:3,4);
    end
    
    %% update plane segmentation image part
    
    axes(ha1); cla;
    plot_segmentation_image(R_cM, sNV, sPP, imageCurForMW, optsMWO);
    title('plane segmentation image');
    
    %% update unit sphere in SO(3) part
    
    axes(ha2); cla;
    plot_sphere_compass(R_cM, sNV, optsMWO);
    view(-1, -71);
    title('unit sphere in SO(3)');
    
    %% update 3D trajectory part
    
    axes(ha3); cla;
    % draw moving trajectory
    plot3( p_gc_OPVO(1,1:imgIdx), p_gc_OPVO(2,1:imgIdx), p_gc_OPVO(3,1:imgIdx), 'm', 'LineWidth', 2 ); hold on; grid on; axis equal;
    plot3( p_gc_true(1,1:imgIdx), p_gc_true(2,1:imgIdx), p_gc_true(3,1:imgIdx), 'k', 'LineWidth', 2 );
    
    % draw camera body and frame
    plot_inertial_frame(0.5);
    RgcOPVO_current = T_gc_OPVO{imgIdx}(1:3,1:3);
    pgcOPVO_current = T_gc_OPVO{imgIdx}(1:3,4);
    plot_camera_frame(RgcOPVO_current, pgcOPVO_current, imageCurForMW, 1.3, 'm'); hold off;
    refresh; pause(0.01);
    
    %% save current figure
    
    if (toSave)
        % save directory for MAT data
        SaveDir = [datasetPath '/BMVC2017'];
        if (~exist( SaveDir, 'dir' ))
            mkdir(SaveDir);
        end
        
        % save directory for images
        SaveImDir = [SaveDir '/OPVO'];
        if (~exist( SaveImDir, 'dir' ))
            mkdir(SaveImDir);
        end
        
        pause(0.1); refresh;
        saveImg = getframe(h);
        imwrite(saveImg.cdata , [SaveImDir sprintf('/%06d.png', imgIdx)]);
    end
end
