clear;
clc;

%% kinect深度图像采集
% % 创建深度传感器对象
% Depth = videoinput('kinect',2);
% 
% % 设置触发器帧数为1
% Depth.FramesPerTrigger = 1;
% % 触发重复次数设置为1
% Depth.TriggerRepeat = 1;
% % 配置kinect手动触发传感器
% triggerconfig(Depth,'manual');
% % 启动深度传感器对象
% start(Depth);
% % 触发摄像头获取数据
% trigger(Depth);
% % 这里仅获取点云数据，抛弃其他数据ts_depth，metaData_Depth
% [imgDepth, ~, ~] = getdata(Depth);
% 
% % 从kinect深度对象提取点云数据
% ptCloud = pcfromkinect(Depth,imgDepth);
% % 停止kinect相机
% stop(Depth);
% pcwrite(ptCloud,'test2.ply','Encoding','binary');

%% 初始化点云显示框
% 设置垂直轴为y轴，垂直轴方向down

% player = pcplayer(ptCloud.XLimits, ptCloud.YLimits, ptCloud.ZLimits,...
%     'VerticalAxis', 'y', 'VerticalAxisDir', 'down');
% 
% % 设置坐标轴
% xlabel(player.Axes, 'X (m)');
% ylabel(player.Axes, 'Y (m)');
% zlabel(player.Axes, 'Z (m)');
% % 显示点云数据
% view(player, ptCloud);

%% 点云文件读取
ptCloud = pcread('test2.ply');

%% 点云预处理
% % 划分感兴趣区域(m)，只检测此范围内点云
% xBound = 4;
% yBound = 3;
% zBound_min = 0.5;
% zBound_max = 3.5;
% xlimits = [-xBound, xBound];
% ylimits = [-yBound, yBound];
% zlimits = [zBound_min, zBound_max];
% 
% indices = find(ptCloud.Location(:, 1) >= -xBound ...
%              & ptCloud.Location(:, 1) <=  xBound ...
%              & ptCloud.Location(:, 2) >= -yBound ...
%              & ptCloud.Location(:, 2) <=  yBound ...
%              & ptCloud.Location(:, 3) >=  zBound_min ...
%              & ptCloud.Location(:, 3) <=  zBound_max );
% 
% patch = select(ptCloud, indices);


patch = ptCloud;
% no_noise = ptCloud;

% 删除离群点，NumNeighbors是最近邻点数量，Threshold是离群点阈值
[no_noise,inlierIndices,outlierIndices] = pcdenoise(patch,...
    'NumNeighbors', 40,'Threshold', 0.05);

% 下采样部分（为了加速使用，可以注释）
percentage = 0.4;
no_noise = pcdownsample(no_noise, 'random',percentage);

%% 分割地平面和平面上的物体
% 找到地面平面并移除地面平面点。使用RANSAC算法检测和匹配地面平面。
% 平面的法线方向应大致沿 y 轴向上指向。所有被划分到地面的点必须
% 在拟合的地面平面的2厘米以内。

maxDistance = 0.02; % in meters
referenceVector = [0, 1, 0];
[~, Ground, outliers] = pcfitplane(no_noise, maxDistance, referenceVector);


% 选择不属于地平面一部分的点。
pcWithoutGround = select(no_noise, outliers);

%% 感兴趣区域划分
% 除去地面标识点后，检索x=[-0.5, 0.5] y=[-0.5 0.5] z=[1, 2]范围内的点
roi = [-1 0.8 -1 1 1 2];
Maybe_target_index = findPointsInROI(pcWithoutGround,roi);
Maybe_target = select(pcWithoutGround, Maybe_target_index);
%% 基于欧氏距离进行分割
% 设定最小欧氏距离，分割后提取所有类别中点数量最多的当做目标
minDistance = 0.03;
% 根据欧氏距离进行分割
[labels,~] = pcsegdist(Maybe_target,minDistance);

% 对labels中元素出现次数排序，左边次数从上到下递减，右边对应的标签
k=sort(labels');
w=diff(find([1 diff(k)==1 1]));
LabelsSeq = sortrows([w',unique(k)'], 'descend');

% 分割结果中出现最多的两个类别当做物体
target_index_1 = find(labels==LabelsSeq(1,2));
target_1 = select(Maybe_target, target_index_1);

target_index_2 = find(labels==LabelsSeq(2,2));
target_2 = select(Maybe_target, target_index_2);
% pcshow(target);

%% 计算OriBoundingBox
cornerPoints_1 = calc_OriBoundingBox(double(target_1.Location));
boxLine_1 = OBB_box_line(cornerPoints_1);

cornerPoints_2 = calc_OriBoundingBox(double(target_2.Location));
boxLine_2 = OBB_box_line(cornerPoints_2);

%% 颜色标记
% 设置用于标记点的颜色表。
colors = [0 0 1; ...  % 蓝色为未标记的点(0 0 1); 指定为[R，G，B]
          0 1 0; ...  % 绿色的为地面平面点(0 1 0)
          1 0 0; ...  % 红色的为物体1
          1 0 1; ...  % 紫色的为物体2
          0 0 0];     % 黑色的为自我附近点(0 0 0)
      
blueIdx  = 0; % 整个点云最初是蓝色的
greenIdx = 1; % 绿色标签（地面）
redIdx   = 2; % 红色标签（物体1）
purpleIdx = 3; % 紫色标签（物体2）
blackIdx = 4; % 黑色标签（边框）
% 将颜色标签附加到点云中的每个点。使用绿色显示地面平面和红色的物体
labelSize   = [no_noise.Count, 1];
colorLabels = zeros(labelSize, 'single');

boxLineLabels_1 = blackIdx*ones([boxLine_1.Count, 1], 'single');
boxLineLabels_2 = blackIdx*ones([boxLine_2.Count, 1], 'single');


% 标出地平面点。
colorLabels(Ground) = greenIdx;
% 根据标签对疑似目标点进行染色(索引传递到no_noise）
colorLabels((outliers(Maybe_target_index(target_index_1)))) = redIdx;
colorLabels((outliers(Maybe_target_index(target_index_2)))) = purpleIdx;


%% 结果显示
% 将所有标记的点绘制到点云播放器中。使用前面设置的数字颜色标签。
player_3 = pcplayer(no_noise.XLimits, no_noise.YLimits, no_noise.ZLimits,...
    'VerticalAxis', 'y', 'VerticalAxisDir', 'down');
colormap(player_3.Axes, colors)
% points1(inPlanePointIndices, :) = [];
% 直接将边框点云和标签加在后面一起显示
view(player_3, [no_noise.Location;boxLine_1.Location;boxLine_2.Location],...
    [colorLabels;boxLineLabels_1;boxLineLabels_2]);
title(player_3.Axes, 'Segmented Point Cloud');
