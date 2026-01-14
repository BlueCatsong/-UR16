close all; clc; clear;

% =========================================================================
%                         无机械臂-实物扫描模式 (修正版)
% 1. 已移除 STL 模型导入，改为设定“扫描工作区”。
% 2. 增强了相机报错信息的显示。
% =========================================================================

%% --- [1] RealSense 相机初始化 ---
%% --- [1] RealSense 相机初始化 (修正版) ---
fprintf('正在初始化 RealSense 相机...\n');
try
    % 创建管道
    pipe = realsense.pipeline();
    cfg = realsense.config();
    
    % --- 修改点 1: 降低帧率到 15 FPS，提高 USB 兼容性 ---
    % 如果你的 USB 连接不稳定，30 FPS 容易导致 "Frame didn't arrive"
    cfg.enable_stream(realsense.stream.depth, 640, 480, realsense.format.z16, 15);
    cfg.enable_stream(realsense.stream.color, 640, 480, realsense.format.rgb8, 15);
    
    % 开始采集
    profile = pipe.start(cfg);
    
    % 获取深度比例
    depth_sensor = profile.get_device().first('depth_sensor');
    depth_scale = depth_sensor.get_depth_scale();
    fprintf('相机初始化成功! Depth Scale: %f\n', depth_scale);
    
    % --- 修改点 2: 柔和预热 ---
    % 之前的循环读取容易超时，改为纯等待让相机自动调节曝光
    fprintf('正在预热相机 (3秒)... \n');
    pause(3); 
    
    % 尝试丢弃第一帧（清空缓存），如果失败也不报错
    try
        % 增加超时时间到 5000ms
        pipe.wait_for_frames(5000); 
    catch
        fprintf('预热帧读取超时，跳过，继续执行主程序...\n');
    end
    
catch e
    warning('RealSense 相机启动失败！');
    fprintf('错误详情: %s\n', e.message);
    % 如果是 Frame didn't arrive，通常是 USB 问题
    if contains(e.message, 'Frame didn''t arrive')
        fprintf('\n======== 故障排查建议 ========\n');
        fprintf('1. 请务必插入蓝色的 USB 3.0 接口，不要用 USB 2.0。\n');
        fprintf('2. 请检查 USB 线是否松动，或更换线缆。\n');
        fprintf('3. 确保 RealSense Viewer 软件已关闭。\n');
        fprintf('=============================\n');
    end
    error('测试终止。');
end
% 输入输出设置
inputDir = 'inputs';
outputDir = 'outputs_real_scan'; 

% Sensor parameters (修正补全版)
sensor = [];
sensor.position =       [435 435 400]; % 初始相机位置
sensor.rotationMatrix = [0  1  0;      
                         1  0  0;
                         0  0 -1];
sensor.resAz = 640;                    
sensor.resEl = 480;                    
sensor.azRange = [-1.2909/2 1.2909/2]; 
sensor.elRange = [-1.0816/2 1.0816/2]; 
sensor.range = [100 800];              % 有效深度范围 (mm)

% --- [修复] 补全缺失的字段，防止 F_generatePData 报错 ---
sensor.optimumDist = 350;         	   % 最佳扫描距离 (mm)
sensor.optimumDist_accuracy = 350;     % (新增) 精度依赖距离，保持一致即可
sensor.optimumDist_sampling = 350;     % (新增) 采样依赖距离，保持一致即可
sensor.type = 'cartesian';             % (新增) 必须指定，否则报错
% ----------------------------------------------------

% 处理参数
desRes = 1.0;                % 采样密度 (mm)，未知物体建议先设大一点，加快速度
safetyDist = 50;             
processingMode = 'vertices'; 
dsMode = 'best';             
nAngTests = 5;               

% 机器人参数 (仅用于避障计算)
robotData = [];
robotData.Links = [180.7; 478.4; 360.0; 174.15; 119.85; 116.55]; 
robotData.Joints = zeros(6,3); 
robotData.robotPrefConf = 0;
robotData.toolParameters = [0 0 0 0 0 0];

%% --- [2] 定义扫描工作区 (替代原 STL 模型) ---
% 我们不再读取 STL，而是定义一个以 [435, 435, 0] 为中心的虚拟盒子
% 这代表了物体所在的“桌子”或“区域”。

workCenter = [435, 435, 0]; % 工作台中心
workSize = [200, 200, 150]; % 长宽高 (mm)，请根据实际物体大小调整
fprintf('设定扫描区域中心: [%.1f, %.1f, %.1f], 范围: %.1f x %.1f mm\n', ...
    workCenter, workSize(1), workSize(2));

% 创建虚拟包围盒用于绘图和边界定义
[sampleTriangulation, lb, ub] = F_createWorkSpaceBox(workCenter, workSize, sensor.optimumDist);

% 创建输出目录
if ~exist(outputDir, 'dir'), mkdir(outputDir); end
inputDir = [strrep(pwd,'\','/') '/' inputDir];
outputDir = [strrep(pwd,'\','/') '/' outputDir];

%% --- [3] 绘图环境初始化 ---
screenSize = get(0, 'ScreenSize');
hf = figure(1); set(hf, 'Position', screenSize);

hSubplot1 = subplot(1,2,1);
% 绘制半透明的工作区盒子，提示用户物体应该放在这里
hSample = patch('Faces',sampleTriangulation.ConnectivityList,'Vertices',sampleTriangulation.Points,...
                'edgecolor',[0.5 0.5 0.5],'facecolor',[0.9 0.9 0.9],'facealpha',0.2,'DiffuseStrength',0.8);
material(hSample,'dull');
hz=zoom; setAxes3DPanAndZoomStyle(hz,hSubplot1,'camera');
axis image; grid on; hold on; view(60,30); light; lighting gouraud;
xlabel('X'); ylabel('Y'); zlabel('Z');
title('Scan Workspace (Red Box = Target Area)');

hSubplot2 = subplot(1,2,2);
hMesh = trisurf([nan nan nan],nan,nan,nan,0,'FaceColor','interp','EdgeColor',[0.25 0.25 0.25],'facealpha',0.75);
hTitle = title('Visited poses: 0','fontsize',14,'fontweight','bold');
axis image; grid on; view(60,30);
title('Real-time Reconstruction Result');

%% --- [4] 初始化变量 ---
[rawWriteID,fDetWriteID,iDsData] = F_initSaveAndDownsample(outputDir);
dsCubeSide = sqrt(1/(sqrt(2)*desRes));
allSensors = [];
k = 0;
hSensor = [];

%% === 主循环 ===
while 1
    k=k+1;
    
    % 更新仿真视图
    delete(hSensor);
    poseVis = rigid3d(sensor.rotationMatrix', sensor.position);
    hSensor = plotCamera('Parent',hSubplot1,'AbsolutePose',poseVis,'Opacity',0.3,'Size',20,'color',[0 1 0]);
    drawnow;
    
    % 提示用户
    fprintf('\n=========================================\n');
    fprintf('第 %d 次采集。\n', k);
    fprintf('算法建议位置 (mm): [%.1f, %.1f, %.1f]\n', sensor.position);
    
    % 计算 RP Y 角度用于显示
    eul = rotm2eul(sensor.rotationMatrix);
    fprintf('算法建议角度 (RPY): %.2f, %.2f, %.2f\n', eul);
    
    fprintf('>>> 请手动对齐相机，然后按 Enter 采集 (Ctrl+C 退出)...');
    % pause; % 取消注释此行以手动确认
    fprintf(' 正在采集...\n');
    
    % --- 从真实相机获取点云 ---
    try
        % 尝试采集
        rawCloudCam = F_getRealSenseData(pipe, depth_scale);
        
        if isempty(rawCloudCam)
             warning('采集到的点云为空！可能距离过近(<10cm)或过远。');
             continue;
        end
    catch ME
        % --- 捕捉并打印详细错误 ---
        warning('采集发生错误！');
        fprintf('错误 ID: %s\n', ME.identifier);
        fprintf('错误信息: %s\n', ME.message);
        fprintf('尝试重置连接...\n');
        pause(2);
        continue;
    end
    
    % --- 坐标变换: 相机系 -> 世界系 ---
    % 假设当前相机就在算法规划的位置
    pts_cam = rawCloudCam(:, 1:3); % mm
    colors = rawCloudCam(:, 4:6);
    
    % 变换
    pts_world = (sensor.rotationMatrix * pts_cam')' + sensor.position;
    
    % 简单的ROI过滤：只保留工作区附近的点，去除背景杂乱点
    % 这一步对于扫描未知物体很重要，防止扫描到地板或远处的墙
    roi_margin = 100; % 允许超出框一点点
    valid_idx = pts_world(:,1) > (lb(1)-roi_margin) & pts_world(:,1) < (ub(1)+roi_margin) & ...
                pts_world(:,2) > (lb(2)-roi_margin) & pts_world(:,2) < (ub(2)+roi_margin) & ...
                pts_world(:,3) > (lb(3)-roi_margin) & pts_world(:,3) < (ub(3)+roi_margin);
            
    if sum(valid_idx) < 100
        warning('有效点数量过少，请检查相机是否对准了工作台中心 [435,435,0]。');
        continue;
    end
    
    pts_world = pts_world(valid_idx, :);
    colors = colors(valid_idx, :);
    d_vals = pts_cam(valid_idx, 3);
    
    % 构造 Detection Vectors
    view_dir_world = sensor.rotationMatrix(:, 3)';
    det_vecs = view_dir_world .* d_vals;
    
    newData = [pts_world, det_vecs, colors];
    
    % 保存与处理
    allSensors{k} = sensor;
    [rawWriteID,fDetWriteID,mRData,mPData,iRData,iPData,iDsData,iClusters] = ...
        F_saveAndDownsample(rawWriteID,fDetWriteID,newData,iDsData,sensor,dsCubeSide,dsMode,outputDir);
    
    % 泊松重建
    mesh = [];
    try
        [mesh.vertices,mesh.faces] = F_poissonReconstruction(outputDir,'dsData.ply',outputDir,'reconShape.ply');
    catch
        warning('重建步骤失败，可能是点云不足，继续下一轮采集。');
        mesh.vertices = [];
    end
    
    % 绘图与规划
    if ~isempty(mesh.vertices) && ~isempty(mesh.faces)
        [mesh.facesCentres,mesh.facesNormals] = F_getTriangleNormals(mesh.vertices,mesh.faces);
        [~,mesh.verticesNormals] = F_getNodeNormals(mesh.vertices,mesh.faces,mesh.facesCentres,mesh.facesNormals);
        [mesh.facesArea,mesh.verticesArea,mesh.totalArea] = F_meshArea(mesh.faces,mesh.vertices);
        
        [achievedSampling,achievedDensity,achievedCentrality] = F_currentDensity(allSensors,mesh,processingMode);
        
        % 更新显示
        set(hMesh,'Faces',mesh.faces,'Vertices',mesh.vertices,'CData',achievedDensity,'FaceVertexAlphaData',0.75);
        drawnow;
        
        % 规划下一个视角
        initialTestLocations = F_initialTestLocations(sensor.optimumDist,mesh,achievedSampling,desRes,nAngTests,20,lb,ub,robotData,processingMode);
        
        if size(initialTestLocations,1) > 0
            iLowSampling = find(achievedSampling < desRes);
            % 如果所有点都采样达标，强制找最稀疏的
            if isempty(iLowSampling)
                iLowSampling = (1:length(achievedSampling))';
            end
            
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
            disp('没有更多可行视点。');
        end
    else
        % 如果还没有生成网格，只是简单地绕着中心转一下
        warning('暂无重建模型，执行默认旋转策略...');
        current_angle = atan2(sensor.position(2)-workCenter(2), sensor.position(1)-workCenter(1));
        new_angle = current_angle + deg2rad(30); % 旋转30度
        radius = 350;
        sensor.position(1) = workCenter(1) + radius * cos(new_angle);
        sensor.position(2) = workCenter(2) + radius * sin(new_angle);
        % 保持朝向中心
        % (这里简单处理，实际需要计算旋转矩阵指向中心)
    end
end

F_closeSaveAndDownsample(rawWriteID,fDetWriteID);
pipe.stop();

% === 内部辅助函数 ===
function [tr, lb, ub] = F_createWorkSpaceBox(center, size_box, optDist)
    % 生成一个简单的立方体 triangulation 作为工作区示意
    dx = size_box(1)/2; dy = size_box(2)/2; dz = size_box(3)/2;
    nodes = [center(1)-dx, center(2)-dy, center(3);      % 1
             center(1)+dx, center(2)-dy, center(3);      % 2
             center(1)+dx, center(2)+dy, center(3);      % 3
             center(1)-dx, center(2)+dy, center(3);      % 4
             center(1)-dx, center(2)-dy, center(3)+dz*2; % 5 (Top)
             center(1)+dx, center(2)-dy, center(3)+dz*2; % 6
             center(1)+dx, center(2)+dy, center(3)+dz*2; % 7
             center(1)-dx, center(2)+dy, center(3)+dz*2];% 8
    
    % 定义立方体的面 (triangles)
    facets = [1 2 3; 1 3 4; 
              1 2 6; 1 6 5; 
              2 3 7; 2 7 6;
              3 4 8; 3 8 7; 
              4 1 5; 4 5 8; 
              5 6 7; 5 7 8];
          
    tr = triangulation(facets, nodes);
    
    % 计算边界 (Lower Bound / Upper Bound) 用于规划搜索空间
    % 搜索空间应比盒子大，加上 optimumDist
    lb = [min(nodes(:,1))-optDist, min(nodes(:,2))-optDist, 50, -pi, -pi, -pi];
    ub = [max(nodes(:,1))+optDist, max(nodes(:,2))+optDist, center(3)+optDist+200, pi, pi, pi];
end