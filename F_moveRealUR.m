function F_moveRealUR(t_client, target_pose_mat)
% F_moveRealUR 发送 movel 指令给 UR 机械臂
% t_client: tcpclient 对象
% target_pose_mat: 4x4 齐次变换矩阵 (单位：米)

    % 1. 提取位置 (x, y, z)
    pos = target_pose_mat(1:3, 4);
    
    % 2. 提取旋转矩阵并转为 轴角 (Axis-Angle / Rotation Vector)
    R = target_pose_mat(1:3, 1:3);
    rot_vec = F_rotm2axisangle(R); % 需要一个旋转矩阵转轴角的函数
    
    % 3. 拼接 URScript 指令
    % movel(p[x,y,z,rx,ry,rz], a=0.5, v=0.2)
    cmd = sprintf('movel(p[%f,%f,%f,%f,%f,%f], a=0.5, v=0.2)\n', ...
                  pos(1), pos(2), pos(3), ...
                  rot_vec(1), rot_vec(2), rot_vec(3));
                  
    % 4. 发送指令
    write(t_client, uint8(cmd));
    
    % 5. (可选) 阻塞等待
    % 实际项目中建议读取 robot 状态判断是否运动完成，这里简化为根据距离估算时间
    % 或者在外部加 pause
    pause(2.0); % 简单延时等待运动结束
end

function rv = F_rotm2axisangle(R)
    % 简单的旋转矩阵转轴角向量 (Rodrigues' rotation formula 逆变换)
    theta = acos((trace(R) - 1) / 2);
    if theta < 1e-4
        rv = [0, 0, 0];
    else
        axis = [R(3,2)-R(2,3), R(1,3)-R(3,1), R(2,1)-R(1,2)];
        axis = axis / norm(axis);
        rv = axis * theta;
    end
end