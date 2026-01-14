function robotPose = F_getRobotPose(point, viewVec, angle, robotData)
% F_getRobotPose 计算机器人末端位姿以对准目标点
% Inputs:
%   point: [1x3] 目标点坐标 (x, y, z)
%   viewVec: [1x3] 视线向量 (通常是法向量的负方向)
%   angle: [scalar] 绕视线轴的旋转角度 (弧度)
%   robotData: 机器人参数 (此处用于兼容接口，本函数主要做几何计算)
% Output:
%   robotPose: [1x6] [x, y, z, rx, ry, rz] (位置 + 欧拉角)
%              注意：返回的角度单位是【度】，因为 F_initialTestLocations 后续使用了 'deg' 标志

    % 1. 归一化视线向量 (这将是传感器/工具的 Z 轴)
    z_axis = viewVec / norm(viewVec);
    
    % 2. 构建坐标系
    % 定义一个临时的“上”方向来计算 X 轴
    if abs(z_axis(3)) < 0.99
        up = [0 0 1];
    else
        up = [0 1 0];
    end
    
    % X 轴垂直于 Z 轴和 Up 向量
    x_axis = cross(up, z_axis);
    x_axis = x_axis / norm(x_axis);
    
    % Y 轴由 Z 和 X 叉乘得到
    y_axis = cross(z_axis, x_axis);
    
    % 3. 基础旋转矩阵 (Z轴对准viewVec)
    R_base = [x_axis', y_axis', z_axis'];
    
    % 4. 应用绕 Z 轴的自旋 (angle 是弧度)
    R_spin = [cos(angle) -sin(angle) 0;
              sin(angle)  cos(angle) 0;
              0           0          1];
              
    R_final = R_base * R_spin;
    
    % 5. 转换为欧拉角 (ZYX 顺序)
    % 重要：转换为【度】，以匹配 F_initialTestLocations 第 127 行的 'deg' 参数
    eul = rotm2eul(R_final, 'ZYX'); 
    eul_deg = rad2deg(eul);
    
    % 6. 组合结果
    robotPose = [point, eul_deg];
end