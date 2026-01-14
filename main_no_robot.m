close all; clc; clear;

% =========================================================================
%                 无机械臂-实物扫描模式 (修复索引报错版)
% 1. 修复了采集失败跳过时导致的 allSensors 索引空缺报错问题。
% 2. 只有成功采集到数据后，扫描计数器才会增加。
% 3. 保留了实时预览和彩色点云显示功能。
% =========================================================================

%% --- [1] RealSense 相机初始化 ---
fprintf('正在初始化 RealSense 相机...\n');
try
    pipe = realsense.pipeline();
    cfg = realsense.config();
    % 使用 15FPS 提高稳定性
    cfg.enable_stream(realsense.stream.depth, 640, 480, realsense.format.z16, 15);
    cfg.enable_stream(realsense.stream.color, 640, 480, realsense.format.rgb8, 15);
    profile = pipe.start(cfg);
    
    depth_sensor = profile.get_device().first('depth_sensor');
    depth_scale = depth_sensor.get_depth_scale();
    fprintf('相机初始化成功! Depth Scale: %f\n', depth_scale);
    
    % 预热
    fprintf('正在预热相机 (2秒)...\n');
    pause(2);
    try, pipe.wait_for_frames(2000); catch, end

catch e
    warning('RealSense 相机初始化失败！');
    fprintf('错误详情: %s\n', e.message);
    error('请检查 USB 连接。');
end

% 输入输出设置
inputDir = 'inputs';
outputDir = 'outputs_real_scan'; 

% 传感器参数
sensor = [];
sensor.position =       [435 435 400]; 
sensor.rotationMatrix = [0  1  0;      
                         1  0  0;
                         0  0 -1];
sensor.resAz = 640;                    
sensor.resEl = 480;                    
sensor.azRange = [-1.2909/2 1.2909/2]; 
sensor.elRange = [-1.0816/2 1.0816/2]; 
sensor.range = [100 800];              
sensor.optimumDist = 350;         	   
sensor.optimumDist_accuracy = 350;     
sensor.optimumDist_sampling = 350;     
sensor.type = 'cartesian';             

% 处理参数
desRes = 1.0;                
safetyDist = 50;             
processingMode = 'vertices'; 
dsMode = 'best';             
nAngTests = 5;               

% 机器人参数
robotData = [];
robotData.Links = [180.7; 478.4; 360.0; 174.15; 119.85; 116.55]; 
robotData.Joints = zeros(6,3); 
robotData.robotPrefConf = 0;
robotData.toolParameters = [0 0 0 0 0 0];

%% --- [2] 定义扫描工作区 ---
workCenter = [435, 435, 0]; 
workSize = [250, 250, 200]; 
[sampleTriangulation, lb, ub] = F_createWorkSpaceBox(workCenter, workSize, sensor.optimumDist);

if ~exist(outputDir, 'dir'), mkdir(outputDir); end
inputDir = [strrep(pwd,'\','/') '/' inputDir];
outputDir = [strrep(pwd,'\','/') '/' outputDir];

%% --- [3] 绘图环境初始化 ---
screenSize = get(0, 'ScreenSize');
hf = figure(1); set(hf, 'Position', screenSize);

hSubplot1 = subplot(1,2,1);
hSample = patch('Faces',sampleTriangulation.ConnectivityList,'Vertices',sampleTriangulation.Points,...
                'edgecolor',[0.5 0.5 0.5],'facecolor',[0.9 0.9 0.9],'facealpha',0.1,'DiffuseStrength',0.8);
material(hSample,'dull');
hz=zoom; setAxes3DPanAndZoomStyle(hz,hSubplot1,'camera');
axis image; grid on; hold on; view(60,30); light; lighting gouraud;
xlabel('X'); ylabel('Y'); zlabel('Z');
title('Simulation View (Overlay: Real RGB Cloud)');

hSubplot2 = subplot(1,2,2);
hMesh = trisurf([nan nan nan],nan,nan,nan,0,'FaceColor','interp','EdgeColor','none','facealpha',0.75);
hTitle = title('Reconstruction (Density Map)','fontsize',14,'fontweight','bold');
axis image; grid on; view(60,30);
colormap(hSubplot2, jet); 
colorbar(hSubplot2);

%% --- [4] 初始化变量 ---
[rawWriteID,fDetWriteID,iDsData] = F_initSaveAndDownsample(outputDir);
dsCubeSide = sqrt(1/(sqrt(2)*desRes));
allSensors = {}; % 显式初始化为空 Cell 数组
k = 0;           % 有效帧计数器
loop_cnt = 0;    % 循环尝试计数器
hSensor = [];
hRealCloud = [];

%% === 主循环 ===
while 1
    loop_cnt = loop_cnt + 1;
    
    % 更新仿真相机位置 (显示当前计划采集的位置)
    delete(hSensor);
    poseVis = rigid3d(sensor.rotationMatrix', sensor.position);
    hSensor = plotCamera('Parent',hSubplot1,'AbsolutePose',poseVis,'Opacity',0.3,'Size',30,'color',[0 1 0]);
    drawnow;
    
    fprintf('\n=========================================\n');
    fprintf('准备采集第 %d 个有效视角 (尝试次数: %d)\n', k+1, loop_cnt);
    fprintf('建议位置: [%.1f, %.1f, %.1f]\n', sensor.position);
    
    % --- 实时预览 ---
    F_livePreview(pipe); 
    
    fprintf(' 正在采集数据...\n');
    
    % --- 1. 从真实相机获取点云 ---
    try
        rawCloudCam = F_getRealSenseData(pipe, depth_scale);
        if isempty(rawCloudCam)
             warning('采集点云为空 (距离可能不合适)，跳过重试。');
             continue; % 跳过，不增加 k
        end
    catch ME
        warning('采集出错: %s', ME.message);
        pause(1); 
        continue; % 跳过，不增加 k
    end
    
    % --- 2. 坐标变换 & ROI 过滤 ---
    pts_cam = rawCloudCam(:, 1:3); 
    colors = rawCloudCam(:, 4:6);
    
    pts_world = (sensor.rotationMatrix * pts_cam')' + sensor.position;
    
    % 过滤工作区外的点
    roi_idx = pts_world(:,1) > lb(1) & pts_world(:,1) < ub(1) & ...
              pts_world(:,2) > lb(2) & pts_world(:,2) < ub(2) & ...
              pts_world(:,3) > lb(3)-100 & pts_world(:,3) < ub(3);
          
    pts_world = pts_world(roi_idx, :);
    colors = colors(roi_idx, :);
    d_vals = pts_cam(roi_idx, 3);
    
    if isempty(pts_world)
        warning('工作区内未检测到有效点云，请调整相机位置或检查工作区定义。');
        continue; % 跳过，不增加 k
    end

    % --- 到这里说明数据有效，可以增加计数器了 ---
    k = k + 1;
    
    % --- 显示彩色点云 ---
    delete(hRealCloud); 
    player = pointCloud(pts_world, 'Color', uint8(colors));
    hRealCloud = pcshow(player, 'Parent', hSubplot1, 'MarkerSize', 50); 
    hold(hSubplot1, 'on'); 
    title(hSubplot1, sprintf('Captured RGB Cloud (Scan %d)', k));
    drawnow;
    
    % --- 构造与保存数据 ---
    view_dir_world = sensor.rotationMatrix(:, 3)';
    det_vecs = view_dir_world .* d_vals;
    newData = [pts_world, det_vecs, colors];
    
    % 保存 allSensors (修复核心：此时 k 是连续的)
    allSensors{k} = sensor;
    
    [rawWriteID,fDetWriteID,mRData,mPData,iRData,iPData,iDsData,iClusters] = ...
        F_saveAndDownsample(rawWriteID,fDetWriteID,newData,iDsData,sensor,dsCubeSide,dsMode,outputDir);
    
    % --- 泊松重建 ---
    mesh = [];
    try
        [mesh.vertices,mesh.faces] = F_poissonReconstruction(outputDir,'dsData.ply',outputDir,'reconShape.ply');
    catch
        warning('重建失败，跳过本次。');
    end
    
    % --- 密度计算与规划 ---
    if ~isempty(mesh.vertices) && ~isempty(mesh.faces)
        [mesh.facesCentres,mesh.facesNormals] = F_getTriangleNormals(mesh.vertices,mesh.faces);
        [~,mesh.verticesNormals] = F_getNodeNormals(mesh.vertices,mesh.faces,mesh.facesCentres,mesh.facesNormals);
        [mesh.facesArea,mesh.verticesArea,mesh.totalArea] = F_meshArea(mesh.faces,mesh.vertices);
        
        % 这里调用 F_currentDensity 不会再报错了
        [achievedSampling,achievedDensity,achievedCentrality] = F_currentDensity(allSensors,mesh,processingMode);
        
        % 更新显示
        set(hMesh,'Faces',mesh.faces,'Vertices',mesh.vertices,'CData',achievedDensity,'FaceVertexAlphaData',0.75);
        title(hSubplot2, sprintf('Reconstruction Density (Scan %d)', k));
        drawnow;
        
        % 规划下一视角
        initialTestLocations = F_initialTestLocations(sensor.optimumDist,mesh,achievedSampling,desRes,nAngTests,20,lb,ub,robotData,processingMode);
        
        if size(initialTestLocations,1) > 0
            iLowSampling = find(achievedSampling < desRes);
            if isempty(iLowSampling), iLowSampling = (1:length(achievedSampling))'; end
            
            fun = @(x) F_predictDensity([x(1) x(2) x(3) x(4) x(5) x(6)],...
                                        sensor,mesh,achievedSampling,...
                                        achievedCentrality,iLowSampling,...
                                        desRes,robotData,safetyDist,...
                                        processingMode);
            options = optimoptions('surrogateopt','InitialPoints',initialTestLocations,'Display','off','MaxFunctionEvaluations',50,'UseParallel',false);
            [x,~] = surrogateopt(fun,lb,ub,options);
            
            sensor.position = [x(1) x(2) x(3)];
            sensor.rotationMatrix = F_eul2rotm([x(4) x(5) x(6)],'rad');
        else
            disp('没有更多可行视点，结束扫描。');
            break;
        end
    else
        warning('暂无模型，执行旋转策略...');
        current_angle = atan2(sensor.position(2)-workCenter(2), sensor.position(1)-workCenter(1));
        new_angle = current_angle + deg2rad(30);
        radius = 350;
        sensor.position(1) = workCenter(1) + radius * cos(new_angle);
        sensor.position(2) = workCenter(2) + radius * sin(new_angle);
    end
end

F_closeSaveAndDownsample(rawWriteID,fDetWriteID);
pipe.stop();

% === 辅助函数 ===
function F_livePreview(pipe)
    hFig = figure('Name', '【按 Q 键拍照】 RealSense 实时预览', ...
                  'NumberTitle', 'off', 'MenuBar', 'none', 'ToolBar', 'none', ...
                  'Position', [100 100 640 480], 'Color', [0 0 0]);
    figure(hFig); 
    hAx = axes('Parent', hFig, 'Position', [0 0 1 1]);
    hIm = imshow(zeros(480, 640, 3, 'uint8'), 'Parent', hAx);
    text(20, 40, '按 Q 键拍照', 'Color', 'g', 'FontSize', 14, 'FontWeight', 'bold', 'Parent', hAx);
    align = realsense.align(realsense.stream.color);
    drawnow;
    while ishandle(hFig)
        key = get(hFig, 'CurrentCharacter');
        if ~isempty(key) && (key == 'q' || key == 'Q')
            break;
        end
        try
            frames = pipe.wait_for_frames(100);
            aligned_frames = align.process(frames);
            color_frame = aligned_frames.get_color_frame();
            if color_frame.get_width() > 0
                data = color_frame.get_data();
                w = color_frame.get_width(); h = color_frame.get_height();
                img = permute(reshape(data', [3, w, h]), [3, 2, 1]);
                set(hIm, 'CData', img);
                drawnow limitrate;
            end
        catch, end
    end
    if ishandle(hFig), close(hFig); end
end

function [tr, lb, ub] = F_createWorkSpaceBox(center, size_box, optDist)
    dx = size_box(1)/2; dy = size_box(2)/2; dz = size_box(3)/2;
    nodes = [center(1)-dx, center(2)-dy, center(3);      
             center(1)+dx, center(2)-dy, center(3);      
             center(1)+dx, center(2)+dy, center(3);      
             center(1)-dx, center(2)+dy, center(3);      
             center(1)-dx, center(2)-dy, center(3)+dz; 
             center(1)+dx, center(2)-dy, center(3)+dz; 
             center(1)+dx, center(2)+dy, center(3)+dz; 
             center(1)-dx, center(2)+dy, center(3)+dz];
    facets = [1 2 3; 1 3 4; 1 2 6; 1 6 5; 2 3 7; 2 7 6; 3 4 8; 3 8 7; 4 1 5; 4 5 8; 5 6 7; 5 7 8];
    tr = triangulation(facets, nodes);
    lb = [min(nodes(:,1))-optDist, min(nodes(:,2))-optDist, 50, -pi, -pi, -pi];
    ub = [max(nodes(:,1))+optDist, max(nodes(:,2))+optDist, center(3)+optDist+200, pi, pi, pi];
end