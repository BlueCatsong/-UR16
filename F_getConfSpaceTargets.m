function [ikSolution, usedConf] = F_getConfSpaceTargets(targetPose, robotData)
% F_getConfSpaceTargets - 针对 UR16e 的数值逆运动学求解 (带自碰撞检测)
% 
% Outputs:
%   usedConf: 1 = 有解且无碰撞; 0 = 无解或发生碰撞

    % 1. 初始化机器人模型 (为了速度，建议在 main 中加载一次传进来，但为了兼容性这里内部加载)
    % 使用 try-catch 确保加载成功
    try
        robot = loadrobot('universalUR16e', 'DataFormat', 'row'); % IK 需要 row 或 column，此处保持 row 方便输出
    catch
        robot = rigidBodyTree('DataFormat','row'); 
        % ... (如果需要手动构建 DH 参数兜底，可在此添加，但通常 loadrobot 没问题)
    end

    % 2. 处理目标位姿 [x y z r p y] (单位: mm, deg) -> (m, rad)
    xyz = targetPose(1:3) / 1000;       % 毫米 -> 米
    rpy = deg2rad(targetPose(4:6));     % 度 -> 弧度
    
    rotMat = eul2rotm(rpy, 'ZYX'); 
    tform = trvec2tform(xyz) * rotm2tform(rotMat);

    % 3. 设置数值求解器
    ik = inverseKinematics('RigidBodyTree', robot);
    weights = [1 1 1 1 1 1]; 
    initialGuess = robot.homeConfiguration; % 使用 Home 作为初始猜测

% 4. 求解
    [configSol, solInfo] = ik(robot.BodyNames{end}, tform, weights, initialGuess);

    % 5. --- [修改] 自碰撞检测 (添加 SkippedSelfCollisions 参数) ---
    
    % 检查是否发生自碰撞
    isColliding = checkCollision(robot, configSol, 'IgnoreSelfCollision', 'off', 'SkippedSelfCollisions', 'parent');
    
    % 检查位置误差
    poseError = solInfo.PoseErrorNorm;
    threshold = 0.005; 

    if poseError < threshold && ~isColliding
        usedConf = 1;
        ikSolution = rad2deg(configSol);
    else
        % 随机重试
        initialGuess = randomConfiguration(robot);
        [configSol, solInfo] = ik(robot.BodyNames{end}, tform, weights, initialGuess);
        
        % 再次检查 (同样添加参数)
        isColliding = checkCollision(robot, configSol, 'IgnoreSelfCollision', 'off', 'SkippedSelfCollisions', 'parent');
        
        if solInfo.PoseErrorNorm < threshold && ~isColliding
             usedConf = 1;
             ikSolution = rad2deg(configSol);
        else
             usedConf = 0;
             ikSolution = zeros(1,6);
        end
    end
end