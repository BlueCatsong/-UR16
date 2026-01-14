function cloudData = F_getRealSenseData(pipe, depth_scale)
% F_GETREALSENSEDATA 从 RealSense 相机获取一帧点云数据
% 修复说明：移除了不兼容的 .is_valid() 调用，改为检查图像尺寸。
%
% 输入:
%   pipe:        已启动的 realsense.pipeline 对象
%   depth_scale: 深度传感器的比例因子
%
% 输出:
%   cloudData:   Nx6 矩阵 [x, y, z, r, g, b] (mm, 0-255)

    % 1. 等待数据帧 (增加超时时间以防USB卡顿)
    try
        frames = pipe.wait_for_frames(5000); 
    catch
        warning('RealSense: 等待帧超时');
        cloudData = [];
        return;
    end
    
    % 2. 帧对齐 (深度对齐到彩色)
    try
        align_to = realsense.stream.color;
        align = realsense.align(align_to);
        aligned_frames = align.process(frames);
        
        depth_frame = aligned_frames.get_depth_frame();
        color_frame = aligned_frames.get_color_frame();
    catch
        warning('RealSense: 对齐处理失败');
        cloudData = [];
        return;
    end
    
    % --- [修改核心] 兼容性修复：用宽度检查代替 is_valid() ---
    % 某些版本的 Wrapper 没有 is_valid 方法，但如果帧无效，get_width() 通常返回 0 或报错
    w = 0; h = 0;
    try
        w = color_frame.get_width();
        h = color_frame.get_height();
        d_w = depth_frame.get_width();
        
        if w == 0 || h == 0 || d_w == 0
            cloudData = [];
            return;
        end
    catch
        % 如果连 get_width 都报错，说明帧完全损坏
        cloudData = [];
        return;
    end
    % ----------------------------------------------------
    
    % 3. 获取彩色数据
    % color_data 是 1 x N 的数组
    color_data = color_frame.get_data(); 
    
    % MATLAB Wrapper 返回的 data 通常是列向量，需要 reshape
    % 格式通常是 [3, w, h] 或 [w, h, 3]，取决于 wrapper 版本
    % 标准 SDK wrapper: 3 x width x height
    img_color = permute(reshape(color_data', [3, w, h]), [3, 2, 1]); 
    
    % 4. 计算点云
    pc = realsense.pointcloud();
    pc.map_to(color_frame);
    points = pc.calculate(depth_frame);
    
    % 5. 获取顶点数据
    vertices = points.get_vertices(); 
    
    % 将顶点转换为 MATLAB 矩阵 (N x 3)
    % 注意：wrapper 返回的 vertices 可能是转置的
    verts_m = vertices; 
    if size(verts_m, 2) ~= 3
        verts_m = verts_m';
    end
    
    % 6. 筛选有效点并匹配颜色
    n_points = w * h;
    
    % 确保顶点数量和像素数量一致 (防止 wrapper 版本差异导致的维度不匹配)
    if size(verts_m, 1) ~= n_points
        % 如果不一致，强制尝试调整
        warning('顶点数量(%d)与像素数量(%d)不匹配，跳过此帧', size(verts_m, 1), n_points);
        cloudData = [];
        return;
    end

    % 展平颜色以便索引
    r = reshape(img_color(:, :, 1)', n_points, 1);
    g = reshape(img_color(:, :, 2)', n_points, 1);
    b = reshape(img_color(:, :, 3)', n_points, 1);
    
    % 简单的深度阈值过滤 (0.1m 到 1.5m) - 过滤掉过近和过远的噪点
    % RealSense 在 0 处的值表示无效深度
    valid_mask = verts_m(:, 3) > 0.1 & verts_m(:, 3) < 1.5;
    
    if sum(valid_mask) == 0
        cloudData = [];
        return;
    end
    
    valid_verts = verts_m(valid_mask, :);
    valid_r = double(r(valid_mask));
    valid_g = double(g(valid_mask));
    valid_b = double(b(valid_mask));
    
    % 7. 组合并转换单位 (米 -> 毫米)
    cloudData = zeros(sum(valid_mask), 6);
    cloudData(:, 1:3) = valid_verts * 1000; % m -> mm
    cloudData(:, 4) = valid_r;
    cloudData(:, 5) = valid_g;
    cloudData(:, 6) = valid_b;

end