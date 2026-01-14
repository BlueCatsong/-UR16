function robot = F_loadUR16e()
    try
        robot = loadrobot('universalUR16e', 'DataFormat', 'column');
    catch
        error('请安装 Robotics System Toolbox Robot Library Data。');
    end
end