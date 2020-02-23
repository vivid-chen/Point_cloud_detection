clc;
clear;

% 创建深度传感器对象
Depth = videoinput('kinect',2);

% srcDepth = getselectedsource(Depth);

% 设置触发器帧数为1
Depth.FramesPerTrigger = 1;
% 触发重复次数设置为1
Depth.TriggerRepeat = 1;
% 配置kinect手动触发传感器
triggerconfig(Depth,'manual');
% 启动深度传感器对象
start(Depth);
% 触发摄像头获取数据
trigger(Depth);
% 这里仅获取点云数据，抛弃其他数据ts_depth，metaData_Depth
[imgDepth, ~, ~] = getdata(Depth);

% 从kinect深度对象提取点云数据
ptCloud = pcfromkinect(Depth,imgDepth);
% 初始化点云播放框 设置垂直轴为y轴，垂直轴方向down
player = pcplayer(ptCloud.XLimits, ptCloud.YLimits, ptCloud.ZLimits,...
    'VerticalAxis', 'y', 'VerticalAxisDir', 'down');
% player = pcplayer([-3 3], ptCloud.YLimits, ptCloud.ZLimits,...
%     'VerticalAxis', 'y', 'VerticalAxisDir', 'down');

% 设置坐标轴
xlabel(player.Axes, 'X (m)');
ylabel(player.Axes, 'Y (m)');
zlabel(player.Axes, 'Z (m)');
% 显示点云数据
view(player, ptCloud);


pcwrite(ptCloud,'test.ply','Encoding','binary');
% ptCloud = pcread('test.ply')


% 删除离群点，第一个是最近邻点数量，第二个是离群点阈值
[no_noise,inlierIndices,outlierIndices] = pcdenoise(ptCloud,...
    'NumNeighbors',50,'Threshold',0.01);

% 开始拟合平面
maxDistance = 0.1;%内点到平面标量值的最大距离
referenceVector = [0,1,0];%约束方向
[no_ground,inliers,outliers] = pcfitplane(no_noise,maxDistance,referenceVector);

% 开始聚类-outliers是不带地面的部分
ptCloudWithoutGround = select(no_noise,outliers,'OutputSize','full');
distThreshold = 0.2;
[labels,numClusters] = pcsegdist(ptCloudWithoutGround,distThreshold);

% 为地面点添加额外标签
numClusters = numClusters+1;
labels(inliers) = numClusters;

% 地面染黑
labelColorIndex = labels+1;
pcshow(no_noise.Location,labelColorIndex)
colormap([hsv(numClusters);[0 0 0]])
title('Point Cloud Clusters')

% % 设定最小欧氏距离
% minDistance = 0.02;
% % 根据欧氏距离进行分割
% [labels,numClusters] = pcsegdist(no_noise,minDistance);
% % 根据标签对不同类别的点进行染色
% % pcshow(no_noise.Location,labels);
% 
% % 初始化点云播放框 设置垂直轴为y轴，垂直轴方向down
% player = pcplayer(no_noise.XLimits, no_noise.YLimits, no_noise.ZLimits,...
%     'VerticalAxis', 'y', 'VerticalAxisDir', 'down');
% % player = pcplayer([-3 3], ptCloud.YLimits, ptCloud.ZLimits,...
% %     'VerticalAxis', 'y', 'VerticalAxisDir', 'down');
% 
% % 设置坐标轴
% xlabel(player.Axes, 'X (m)');
% ylabel(player.Axes, 'Y (m)');
% zlabel(player.Axes, 'Z (m)');
% % 显示点云数据 添加labels信息进行染色显示
% view(player, no_noise.Location, labels);



% 停止kinect相机
stop(Depth);

% 激光雷达点云分割地面代码演示
% openExample('vision/ClusterLidarPointCloudBasedOnEuclideanDistanceExample')