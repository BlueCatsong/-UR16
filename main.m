close all; clc; clear;

%This matlab script demonstrates the execution of the autonomous 3D
%reconstruction pipeline, relative to the paper titled: "Autonomous 3D 
%geometry reconstruction through robot-manipulated optical sensors", by 
%C. Mineo, D. Cerniglia, V. Ricotta and B. Reitinger.
%
% Author: Carmelo Mineo
% Department of Engineering, University of Palermo, Viale delle Scienze,
% Edificio 8, 90128 Palermo, Italy.
% email: carmelo.mineo01@unipa.it
% Website: http://www.unipa.it
% March 2021; Last revision: 29-March-2021
% Tested with: Matlab 2020b


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%                       USER-DEFINED INPUTS                           %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% --- [新增] 硬件连接初始化 ---

% 1. 连接 RealSense 相机
% 确保已安装 RealSense SDK 并且 MATLAB 能找到 realsense.* 类
pipe = realsense.pipeline();
cfg = realsense.config();
cfg.enable_stream(realsense.stream.depth, 640, 480, realsense.format.z16, 30);
cfg.enable_stream(realsense.stream.color, 640, 480, realsense.format.rgb8, 30);
profile = pipe.start(cfg);

% 获取深度缩放因子 (Depth Scale)
depth_sensor = profile.get_device().first('depth_sensor');
depth_scale = depth_sensor.get_depth_scale();

% 2. 连接 UR16e 机械臂 (使用 TCP/IP)
% 替换为你的机械臂实际 IP 地址
robot_ip = '192.168.1.100'; 
robot_port = 30003; % 30003 是实时数据端口，30002 也可以用于发送指令
try
    robot_socket = tcpclient(robot_ip, robot_port);
    fprintf('成功连接到机械臂: %s\n', robot_ip);
catch
    error('无法连接到机械臂，请检查 IP 和网络连接。');
end

% 手眼标定矩阵 (非常重要！)
% 需要你提前标定：从“法兰中心(Tool)”到“相机中心(Camera)”的变换矩阵
% 假设相机安装在法兰上，这里需要填入实际值
T_flange_camera = eye(4); 
% 例如：T_flange_camera(1:3,4) = [0.05; 0; 0.1]; % 相机相对于法兰的偏移





% Input and output subfolders
inputDir = 'inputs';
outputDir = 'outputs';

% Sensor parameters
sensor = [];
sensor.position =       [435 435 350]; % Initial pose Cartesian coordinates
sensor.rotationMatrix = [0  1  0;      % Initial pose rotation matrix
                         1  0  0;
                         0  0 -1];

sensor.resAz = 640;                    % Sensor azimutal resolution
sensor.resEl = 480;                    % Sensor elavation resolution
sensor.azRange = [-1.2909/2 1.2909/2]; % Azimutal angle range
sensor.elRange = [-1.0816/2 1.0816/2]; % Elevation angle range
sensor.range = [0 600];             % Sensor depth range [min max]
sensor.optimumDist_accuracy = 200;	% Sensor accuracy-dependant stand-off
sensor.optimumDist_sampling = 200;	% Sensor sampling-dependant stand-off
sensor.optimumDist = 200;         	% Sensor optimum stand-off
sensor.type = 'cartesian';        	% Sensor type

% Data processing parameters
desRes = 0.05;               % Target sampling density (points/mm^2)
safetyDist = 50;             % Safety distance between sensor pose and sample
processingMode = 'vertices'; % Processing method ('vertices' or 'faces')
dsMode = 'best';             % Downsampling mode ('best', 'worst' or 'average')
nAngTests = 5;               % Number of sensor orientations to consider

% Robot data
% robotData = [];     % robotData contains information about the sensor
%                     % manipulator. If it is empty (robotData=[]), no
%                     % kinematic constraint is considered. If pose
%                     % reachbility must be checked, robotData must contain
%                     % the following information:
%                     % robotData.Links: [6×1 double] (length of robot joints)
%                     % robotData.Joints: [6×3 double] (centre of kiematic joints)
%                     % robotData.robotPrefConf: (robot preferred configuration.
%                     %                          If 0 best configuration is
%                     %                          automatically determined)
%                     % robotData.toolParameters: [0 0 0 0 0 0] (tool parameters)
% -------------------------------------------------------------------------
% Robot data for UR16e
% -------------------------------------------------------------------------
robotData = [];
% UR16e 的连杆长度 (单位: mm)
% 注意：这里的数据结构取决于 F_getConfSpaceTargets 内部是如何使用它的。
% 通常简单的几何法需要臂长。
% L1(Base), L2(Upper), L3(Forearm), L4(W1), L5(W2), L6(W3/Flange)
robotData.Links = [180.7; 478.4; 360.0; 174.15; 119.85; 116.55]; 

% 关节中心定义 (6x3 矩阵)
% 原代码注释说是 "centre of kinematic joints"，这通常用于碰撞检测或几何绘图。
% 如果没有具体的 F_getRobotPose 代码，我们暂时将其设为 0 或标准偏移，
% 但最关键的是上面的 .Links 长度。
robotData.Joints = zeros(6,3); 

% 机器人首选配置 (0 = 自动)
robotData.robotPrefConf = 0;

% 工具参数 (TCP Offset: x, y, z, r, p, y)
% 如果你的传感器安装在法兰中心，全为0。如果有支架，请填入相对法兰的偏移。
robotData.toolParameters = [0 0 0 0 0 0];

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%                     PRELIMINAR CALCULATIONS                         %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Load sample and support geometry
% Load sample and support geometry
sampleTriangulation = stlread([inputDir '\Epick_MW_2.stl']);

% Load sample and support geometry
% sampleTriangulation = stlread([inputDir '\sample.stl']);

% --- [新增] 1. 单位修正 (米 -> 毫米) ---
xRange = max(sampleTriangulation.Points(:,1)) - min(sampleTriangulation.Points(:,1));
yRange = max(sampleTriangulation.Points(:,2)) - min(sampleTriangulation.Points(:,2));
zRange = max(sampleTriangulation.Points(:,3)) - min(sampleTriangulation.Points(:,3));
maxDim = max([xRange, yRange, zRange]);

fprintf('当前 STL 模型最大边长: %.4f\n', maxDim);
if maxDim < 10 
    fprintf('检测到模型尺寸过小，推测单位为[米]，正在转换为[毫米]...\n');
    newPoints = sampleTriangulation.Points * 1000;
else
    fprintf('模型尺寸正常(毫米)，无需转换。\n');
    newPoints = sampleTriangulation.Points;
end

% --- [新增] 2. 位置平移 (解决穿模和扫描不到的问题) ---
% 将工件移动到传感器正下方 [435, 435, 0]，且Z轴底面对齐0
% 这样既在机械臂工作区内，又在传感器 600mm 范围内
targetCenter = [435, 435, 0]; 

% 计算当前中心
currentCenter = mean(newPoints);
% 计算底部Z值
minZ = min(newPoints(:,3));

% 平移向量：X/Y 对齐目标中心，Z 对齐地面
translationVector = [targetCenter(1)-currentCenter(1), ...
                     targetCenter(2)-currentCenter(2), ...
                     0 - minZ]; 

fprintf('正在将工件平移至传感器工作区: [%.2f, %.2f, %.2f]...\n', ...
    targetCenter(1), targetCenter(2), 0);

newPoints = newPoints + translationVector;

% 重新构建 triangulation 对象
sampleTriangulation = triangulation(sampleTriangulation.ConnectivityList, newPoints);
% ------------------------------------------------

% Define full directory path for input and output folders. The '/'
% separator is preferred for better cross-platform compatibility
inputDir = [strrep(pwd,'\','/') '/' inputDir];
outputDir = [strrep(pwd,'\','/') '/' outputDir];

% --- [修复] 自动创建 output 文件夹 ---
if ~exist(outputDir, 'dir')
    mkdir(outputDir);
end
% ------------------------------------

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%               PREPARATION OF PLOTTING ENVIRONEMENT                  %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
screenSize = get(0, 'ScreenSize');
hf = figure(1); set(hf, 'Position', get(0, 'Screensize'));

% Subplot for pose simulation
hSubplot1 = subplot(1,2,1);
hSample = patch('Faces',sampleTriangulation.ConnectivityList,...
                'Vertices',sampleTriangulation.Points,...
                'edgecolor','none','facecolor',[0.4 0.6 1],...
                'facealpha',1,'DiffuseStrength',0.8);
material(hSample,'dull');
hz=zoom; setAxes3DPanAndZoomStyle(hz,hSubplot1,'camera');
xlabel('x','FontSize',16);
ylabel('y','FontSize',16);
zlabel('z','FontSize',16,'Rotation',0);
axis image;grid on; hold on;
view(60,30);light; lighting gouraud;

% Subplot for 3D reconstruction result
hSubplot2 = subplot(1,2,2);
hMesh = trisurf([nan nan nan],nan,nan,nan,0,'FaceColor','interp',...
                'EdgeColor',[0.25 0.25 0.25],'facealpha',0.75,...
                'edgealpha',0.25);
hTitle = title('Visited poses: 0  -  Compl. estimate: 0%','fontsize',14,...
               'fontweight','bold');

% Custom colormap definition
mymap = [linspace(1,1,128)'   linspace(0.3,1,128)'   linspace(0.3,0,128)';
         linspace(1,0,128)'   linspace(1,0.8,128)'   zeros(128,1)];

colormap(hSubplot2,mymap);

% Sampling density expected when using the given sensor optimum distance
rho = 0.5*((sensor.resAz*sensor.resEl)/(4*tan(sensor.azRange(2))*tan(sensor.elRange(2))*(sensor.optimumDist^2)));

% Setting the color limits
set(hSubplot2,'CLim',[0 rho]);

% Plot colorbar
hc = colorbar('southoutside','FontSize',10);
hc.Label.String = 'Corrected cumulative sampling density (\lambda)';
hc.Label.FontSize = 12;
xlabel('x','FontSize',16);
ylabel('y','FontSize',16);
zlabel('z','FontSize',16,'Rotation',0);
axis image; grid on;
view(60,30);



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%           INITIALIZATION OF VARIABLES FOR ITERATIVE LOOP            %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Initialize files for data storing and memory mapping
[rawWriteID,fDetWriteID,iDsData] = F_initSaveAndDownsample(outputDir);

% Compute Cartesian lower and upper bounts (lb, ub) for test poses
% Lower bound
lb = [min(sampleTriangulation.Points(:,1))-sensor.optimumDist,...   % min X
      min(sampleTriangulation.Points(:,2))-sensor.optimumDist,...   % min Y
      60,...                                                        % min Z
      -pi,...                                                       % min A
      -pi,...                                                       % min B
      -pi];                                                         % min C

%Upper bound
ub = [max(sampleTriangulation.Points(:,1))+sensor.optimumDist,...   % max X
      max(sampleTriangulation.Points(:,2))+sensor.optimumDist,...   % max Y
      max(sampleTriangulation.Points(:,3))+sensor.optimumDist,...   % max Z
      pi,...                                                        % max A
      pi,...                                                        % max B
      pi];                                                          % max C

dsCubeSide = sqrt(1/(sqrt(2)*desRes));	% Downsampling cube edge size

% Graphical appearance of sensor visualization
hSensor = [];
handles = [];
hxv = [];
hyv = [];
hzv = [];

% --- [修改] 机器人可视化初始化 ---
% --- [修改] 机器人可视化初始化 ---
robotVis = F_loadUR16e();           % 加载标准机器人(米单位)
ikVis = inverseKinematics('RigidBodyTree', robotVis);
weightsVis = [1 1 1 1 1 1];
initialGuessVis = homeConfiguration(robotVis);

% 1. 创建缩放容器 (这个容器自带 1000倍 放大属性)
hRobotScale = hgtransform('Parent', hSubplot1);
set(hRobotScale, 'Matrix', makehgtform('scale', 1000)); 

% 2. 存储机器人图形部件的句柄数组
hRobotParts = []; 
% -------------------------------
% -------------------------------

% Variable to store all sensor poses
allSensors = [];

% Pose counter
k = 0;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%           INCREMENTAL DATA PROCESSING AND 3D RECONSTRUCTION         %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

while 1
    k=k+1;
    
    % Update visualization of sensor position
    delete(hSensor);

    % Update visualization of sensor position
    delete(hSensor);
    
    % --- [新增] 删除上一帧的机器人 ---
    % if ~isempty(hRobot)
    %     delete(hRobot); 
    % end
    % -------------------------------

    pose = rigid3d(sensor.rotationMatrix',...
                   [sensor.position(1)+(-sensor.rotationMatrix(1,3)*30) ...
                    sensor.position(2)+(-sensor.rotationMatrix(2,3)*30) ...
                    sensor.position(3)+(-sensor.rotationMatrix(3,3)*30)]);
    hSensor = plotCamera('Parent',hSubplot1,'AbsolutePose', pose,...
                         'Opacity', 0.3,'Size',20,'color',[0.4 0.4 0.4]);
    
    [patches,edges] = F_syntetic_getSensorVolume(sensor);
    handles = F_syntetic_plotSensorVolume(hSubplot1,handles,patches,edges);
    



% ... (上下文: hSensor = plotCamera...)
    
    % --- [修改] 机器人 IK 求解与姿态优化 (避免自碰撞) ---
    
    % 1. 准备 IK 目标位姿 (毫米 -> 米) [补回了这两行]
    tformMeters = eye(4);
    tformMeters(1:3, 1:3) = sensor.rotationMatrix;
    tformMeters(1:3, 4)   = sensor.position' / 1000; 
    
    % 2. 第一次尝试 (使用上一帧的解作为猜测，保证平滑)
    [configSol, solInfo] = ikVis(robotVis.BodyNames{end}, tformMeters, weightsVis, initialGuessVis);
    
% 3. 检查碰撞 (添加 'SkippedSelfCollisions', 'parent' 以消除警告)
    isColliding = checkCollision(robotVis, configSol, 'IgnoreSelfCollision', 'off', 'SkippedSelfCollisions', 'parent');
    
    % 4. 如果发生碰撞或无解，进行随机重试
    retryCount = 0;
    maxRetries = 10;
    
    while (isColliding || solInfo.PoseErrorNorm > 0.005) && retryCount < maxRetries
        randomGuess = randomConfiguration(robotVis);
        [configSol, solInfo] = ikVis(robotVis.BodyNames{end}, tformMeters, weightsVis, randomGuess);
        
        % 再次检查 (同样添加参数)
        isColliding = checkCollision(robotVis, configSol, 'IgnoreSelfCollision', 'off', 'SkippedSelfCollisions', 'parent');
        retryCount = retryCount + 1;
    end
    
    % 更新下一帧的初始猜测 (如果找到好解就用好的)
    if ~isColliding
        initialGuessVis = configSol;
    end
    
    % --- 绘图部分 (抓取-搬运法) ---
    
    % 5. 删除上一帧的机器人部件
    if ~isempty(hRobotParts)
        delete(hRobotParts);
    end
    
    % 6. 记录当前绘图对象，以便抓取新画出来的机器人
    childrenBefore = allchild(hSubplot1);
    
    % 如果多次尝试还是碰撞，改变颜色为红色警告
    if isColliding
        warningColor = [1 0 0]; % 红色
        % disp(['Warning: Pose ' num2str(k) ' has collision!']); 
    else
        warningColor = [0.8 0.8 0.8]; % 正常的灰色
    end
    
    % 7. 画机器人 (画在坐标轴上)
    show(robotVis, configSol, 'Parent', hSubplot1, ...
         'PreservePlot', true, 'FastUpdate', false, 'Visuals', 'on');
    
    % 8. 找出新生成的部件并搬运到缩放容器中
    childrenAfter = allchild(hSubplot1);
    hRobotParts = setdiff(childrenAfter, childrenBefore);
    set(hRobotParts, 'Parent', hRobotScale);
    
    % 9. 设置颜色 (半透明 + 碰撞变色)
    patches = findobj(hRobotParts, 'Type', 'Patch');
    if ~isempty(patches)
        set(patches, 'FaceColor', warningColor, 'FaceAlpha', 0.5); 
    end
    
    % -----------------------------
    
    
    
    
    % Update visualization of sensor reference system arrows
    delete(hxv);
    delete(hyv);
    delete(hzv);
    
    P1 = sensor.position;
    vector = sensor.rotationMatrix(:,1)';
    P2 = P1 + vector.*100;
    hxv = F_3Darrow(P1,P2,'color',[0.9 0.0 0.0],'stemWidth',4,...
                    'tipWidth',10,'Parent',hSubplot1);
    
    vector = sensor.rotationMatrix(:,2)';
    P2 = P1 + vector.*100;
    hyv = F_3Darrow(P1,P2,'color',[0.0 0.7 0.0],'stemWidth',4,...
                    'tipWidth',10,'Parent',hSubplot1);
    
    vector = sensor.rotationMatrix(:,3)';
    P2 = P1 + vector.*100;
    hzv = F_3Darrow(P1,P2,'color',[0.0 0.0 1],'stemWidth',4,...
                    'tipWidth',10,'Parent',hSubplot1);
    drawnow;
    
    % Simulate acquisition of point cloud from sensor
    % [newData,intersectedFaces] = F_syntetic_receiveCloudFromRGBD(sensor,...
    %       sampleTriangulation.Points,sampleTriangulation.ConnectivityList);


%% --- [修改] 驱动真实硬件采集 ---

% 1. 计算机械臂目标位姿
% sensor.position/rotationMatrix 是优化算出的“相机”目标位姿(World系)
% 我们需要将其转换为“法兰”的目标位姿，并转换为 UR 识别的格式
target_camera_pose = eye(4);
target_camera_pose(1:3, 1:3) = sensor.rotationMatrix;
target_camera_pose(1:3, 4) = sensor.position' / 1000; % 毫米转米 (UR单位是米)

% 计算法兰位姿: T_base_flange = T_base_camera * inv(T_flange_camera)
target_flange_pose = target_camera_pose / T_flange_camera; 

% 2. 发送运动指令给 UR 机械臂
% 这里调用一个自定义函数 (见下文第3部分)
F_moveRealUR(robot_socket, target_flange_pose);

% 3. 等待机械臂稳定 (简单延时，或通过读取 robot_socket 状态判断是否到位)
pause(0.5); 

% 4. RealSense 采集真实点云
% 这里调用一个自定义函数 (见下文第3部分)
% 注意：真实相机采到的点是在“相机坐标系”下的，需要转到“世界坐标系”
real_point_cloud_camera_frame = F_getRealSenseData(pipe, depth_scale);

% 5. 将点云转到世界坐标系 (World Frame)
% 此时机械臂已到达 target_camera_pose
% P_world = R_world_cam * P_cam + T_world_cam
% 注意单位：sensor.position 是 mm，RealSense 采集通常是 m，需要统一到 mm
pts_cam = real_point_cloud_camera_frame(:, 1:3) * 1000; % 米转毫米
pts_world = (sensor.rotationMatrix * pts_cam')' + sensor.position;

% 构造 newData 格式: [x, y, z, vx, vy, vz, R, G, B]
% DetectionVector (vx,vy,vz) 是视线方向，即相机光轴 (Z轴) 在世界系的向量
view_vec_world = sensor.rotationMatrix(:, 3)'; 
num_pts = size(pts_world, 1);
detection_vectors = repmat(view_vec_world, num_pts, 1);
% 计算投影距离 d (参考原仿真代码)
d_vals = pts_cam(:, 3); % 相机系下的 Z 就是距离
detection_vectors = detection_vectors .* d_vals; % 原代码似乎乘以了距离

% 组合数据
% 真实数据颜色通常是 0-255，原代码在 F_saveDsDataAsPLY 里处理了类型
% 确保 real_point_cloud_camera_frame 包含颜色列 4:6
colors = real_point_cloud_camera_frame(:, 4:6); 

newData = [pts_world, detection_vectors, colors];

% ------------------------------------------------


    % Append current sensor to the list of sensors used in previous poses
    allSensors{k} = sensor;
    mPData = [];
    [rawWriteID,fDetWriteID,mRData,mPData,iRData,iPData,iDsData,...
        iClusters] = F_saveAndDownsample(rawWriteID,fDetWriteID,newData,...
        iDsData,sensor,dsCubeSide,dsMode,outputDir);
    
    % Generate mesh through Poisson reconstruction algorithm
    mesh = [];
    [mesh.vertices,mesh.faces] = F_poissonReconstruction(outputDir,...
                                 'dsData.ply',outputDir,'reconShape.ply');
    
    % Compute mesh normals and mesh area
    [mesh.facesCentres,mesh.facesNormals] = F_getTriangleNormals(mesh.vertices,mesh.faces);
    [~,mesh.verticesNormals] = F_getNodeNormals(mesh.vertices,...
                               mesh.faces,mesh.facesCentres,mesh.facesNormals);
    [mesh.facesArea,mesh.verticesArea,mesh.totalArea] = F_meshArea(mesh.faces,mesh.vertices);
    
    
    % Compute sampling densities and centrality factors
    [achievedSampling,achievedDensity,achievedCentrality...
        ] = F_currentDensity(allSensors,mesh,processingMode);
    
    % Update visualization of reconstructed geometry
    if strcmp(processingMode,'vertices')
        vertexAlpha = ones(size(achievedDensity)).*0.75;
        set(hMesh,'FaceAlpha','interp');
        set(hMesh,'AlphaDataMapping','none');
        set(hMesh,'Faces',mesh.faces,'Vertices',mesh.vertices,'CData',...
            achievedDensity.*achievedCentrality,'FaceVertexAlphaData',vertexAlpha);
    else
        hMesh = trisurf([nan nan nan],nan,nan,nan,0,'EdgeColor','none');
        set(hMesh,'FaceVertexAlphaData',1);
        set(hMesh,'Faces',mesh.faces,'Vertices',mesh.vertices);
        set(hMesh,'CData',achievedDensity.*achievedCentrality);
    end
    
    % Compute initial test points for probing the objective function
    initialTestLocations = F_initialTestLocations(sensor.optimumDist,...
        mesh,achievedSampling,desRes,nAngTests,20,lb,ub,robotData,processingMode);
    
    % Compute completion estimate and number of visitable test poses
    expectedPoints = mesh.totalArea*desRes;
    saturatedAchievedSampling = achievedSampling;
    samplingTargetReached = saturatedAchievedSampling>desRes;
    saturatedAchievedSampling(samplingTargetReached) = desRes;
    
    sampledPoints = sum(mesh.verticesArea.*saturatedAchievedSampling);
    completionEstimate = (sampledPoints/expectedPoints)*100;
    visitablePoses = size(initialTestLocations,1);
    
    % Print number of visited poses and completion estimate
    set(hTitle,'String',['Visited poses: ' num2str(k) '  -  Compl. estimate: ' num2str(completionEstimate,'%.0f') '%']);
    
    % Save current figure as .fig and .tiff file
    savefig(hf,[outputDir '\Pose ' num2str(k) '.fig']);
    print([outputDir '\Pose ' num2str(k) '.tiff'],'-dtiff','-r600');
    
    if (visitablePoses>0) && (completionEstimate<99.5) % Verify stopping criteria
        iLowSampling = find(~samplingTargetReached);
        
        % Objective function
        fun = @(x) F_predictDensity([x(1) x(2) x(3) x(4) x(5) x(6)],...
                                    sensor,mesh,achievedSampling,...
                                    achievedCentrality,iLowSampling,...
                                    desRes,robotData,safetyDist,...
                                    processingMode);
        
        % Probe the search space and find best next position among test poses                       
        options = optimoptions('surrogateopt','InitialPoints',initialTestLocations,...
                               'Display','off','MaxFunctionEvaluations',100,...
                               'PlotFcn','surrogateoptplot','UseParallel',false);
        
        [x,fval,exitflag,osur] = surrogateopt(fun,lb,ub,options);
        
        % Update sensor pose
        sensor.position = [x(1) x(2) x(3)];
        sensor.rotationMatrix = F_eul2rotm([x(4) x(5) x(6)],'rad');
    else
        % Break the incremental reconstruction if stopping criteria are met
        break; 
    end
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%                      CLOSE DATA STORING FILES                       %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

F_closeSaveAndDownsample(rawWriteID,fDetWriteID);