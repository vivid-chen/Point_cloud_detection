clc;
clear;

% ������ȴ���������
Depth = videoinput('kinect',2);

% srcDepth = getselectedsource(Depth);

% ���ô�����֡��Ϊ1
Depth.FramesPerTrigger = 1;
% �����ظ���������Ϊ1
Depth.TriggerRepeat = 1;
% ����kinect�ֶ�����������
triggerconfig(Depth,'manual');
% ������ȴ���������
start(Depth);
% ��������ͷ��ȡ����
trigger(Depth);
% �������ȡ�������ݣ�������������ts_depth��metaData_Depth
[imgDepth, ~, ~] = getdata(Depth);

% ��kinect��ȶ�����ȡ��������
ptCloud = pcfromkinect(Depth,imgDepth);
% ��ʼ�����Ʋ��ſ� ���ô�ֱ��Ϊy�ᣬ��ֱ�᷽��down
player = pcplayer(ptCloud.XLimits, ptCloud.YLimits, ptCloud.ZLimits,...
    'VerticalAxis', 'y', 'VerticalAxisDir', 'down');
% player = pcplayer([-3 3], ptCloud.YLimits, ptCloud.ZLimits,...
%     'VerticalAxis', 'y', 'VerticalAxisDir', 'down');

% ����������
xlabel(player.Axes, 'X (m)');
ylabel(player.Axes, 'Y (m)');
zlabel(player.Axes, 'Z (m)');
% ��ʾ��������
view(player, ptCloud);


pcwrite(ptCloud,'test.ply','Encoding','binary');
% ptCloud = pcread('test.ply')


% ɾ����Ⱥ�㣬��һ��������ڵ��������ڶ�������Ⱥ����ֵ
[no_noise,inlierIndices,outlierIndices] = pcdenoise(ptCloud,...
    'NumNeighbors',50,'Threshold',0.01);

% ��ʼ���ƽ��
maxDistance = 0.1;%�ڵ㵽ƽ�����ֵ��������
referenceVector = [0,1,0];%Լ������
[no_ground,inliers,outliers] = pcfitplane(no_noise,maxDistance,referenceVector);

% ��ʼ����-outliers�ǲ�������Ĳ���
ptCloudWithoutGround = select(no_noise,outliers,'OutputSize','full');
distThreshold = 0.2;
[labels,numClusters] = pcsegdist(ptCloudWithoutGround,distThreshold);

% Ϊ�������Ӷ����ǩ
numClusters = numClusters+1;
labels(inliers) = numClusters;

% ����Ⱦ��
labelColorIndex = labels+1;
pcshow(no_noise.Location,labelColorIndex)
colormap([hsv(numClusters);[0 0 0]])
title('Point Cloud Clusters')

% % �趨��Сŷ�Ͼ���
% minDistance = 0.02;
% % ����ŷ�Ͼ�����зָ�
% [labels,numClusters] = pcsegdist(no_noise,minDistance);
% % ���ݱ�ǩ�Բ�ͬ���ĵ����Ⱦɫ
% % pcshow(no_noise.Location,labels);
% 
% % ��ʼ�����Ʋ��ſ� ���ô�ֱ��Ϊy�ᣬ��ֱ�᷽��down
% player = pcplayer(no_noise.XLimits, no_noise.YLimits, no_noise.ZLimits,...
%     'VerticalAxis', 'y', 'VerticalAxisDir', 'down');
% % player = pcplayer([-3 3], ptCloud.YLimits, ptCloud.ZLimits,...
% %     'VerticalAxis', 'y', 'VerticalAxisDir', 'down');
% 
% % ����������
% xlabel(player.Axes, 'X (m)');
% ylabel(player.Axes, 'Y (m)');
% zlabel(player.Axes, 'Z (m)');
% % ��ʾ�������� ���labels��Ϣ����Ⱦɫ��ʾ
% view(player, no_noise.Location, labels);



% ֹͣkinect���
stop(Depth);

% �����״���Ʒָ���������ʾ
% openExample('vision/ClusterLidarPointCloudBasedOnEuclideanDistanceExample')