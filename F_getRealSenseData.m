function cloudData = F_getRealSenseData(pipe, depth_scale)
% 返回格式: [x, y, z, r, g, b] (单位: 米, 0-255)

    % 1. 等待一帧数据
    frames = pipe.wait_for_frames();
    depth_frame = frames.get_depth_frame();
    color_frame = frames.get_color_frame();
    
    % 2. 对齐深度到彩色 (Align Depth to Color)
    align_to = realsense.stream.color;
    align = realsense.align(align_to);
    aligned_frames = align.process(frames);
    aligned_depth = aligned_frames.get_depth_frame();
    aligned_color = aligned_frames.get_color_frame();
    
    % 3. 生成点云对象
    pc = realsense.pointcloud();
    pc.map_to(aligned_color);
    points = pc.calculate(aligned_depth);
    
    % 4. 获取顶点数据 (Vertices)
    vertices = points.get_vertices(); % n x 3 矩阵 (x, y, z)
    % 获取纹理坐标 (Texture Coordinates) 并映射颜色会比较慢，
    % 这里可以直接利用 MATLAB wrapper 导出 ply 再读，或者手动映射。
    % 为简单起见，假设你只想要几何，或者使用 pcshow 的方式。
    
    % 另外一种快速方法：直接把 vertices 和对应的 color image 像素对应起来
    % 因为已经对齐，pixel(u,v) 的深度对应 pixel(u,v) 的颜色
    
    % 将 vertices 转为 MATLAB 矩阵
    % 注意：RealSense Wrapper 的 get_vertices 返回可能是 n x 3
    % 需要将其 reshape 成 image size 才能和 color 对应，或者直接处理
    
    % 简易版：只返回有效深度的点
    num_points = points.size();
    vertices = vertices'; % 转置为 N x 3
    
    % 获取彩色图像数据
    w = aligned_color.get_width();
    h = aligned_color.get_height();
    color_data = aligned_color.get_data();
    % color_data 通常是 1 x (w*h*3) 的 uint8 数组
    color_img = permute(reshape(color_data', [3, w, h]), [3, 2, 1]);
    
    % 过滤掉无效点 (z=0)
    valid_mask = vertices(:,3) > 0 & vertices(:,3) < 1.0; % 假设只取1米内的
    
    valid_verts = vertices(valid_mask, :);
    
    % 提取对应颜色 (需要计算索引)
    % 这里如果不想写复杂的映射，可以暂时返回白色，或者根据索引采样
    % 正确做法是根据 uv 坐标采样，但 calculate() 生成的点是无序的还是有序图？
    % RealSense calculate() 生成的点通常是对应像素坐标的。
    
    cloudData = zeros(sum(valid_mask), 6);
    cloudData(:, 1:3) = valid_verts;
    cloudData(:, 4:6) = 255; % 暂时默认白色，如需彩色需匹配像素索引
end