clear;
clc;

%% kinect���ͼ��ɼ�
% % ������ȴ���������
% Depth = videoinput('kinect',2);
% 
% % ���ô�����֡��Ϊ1
% Depth.FramesPerTrigger = 1;
% % �����ظ���������Ϊ1
% Depth.TriggerRepeat = 1;
% % ����kinect�ֶ�����������
% triggerconfig(Depth,'manual');
% % ������ȴ���������
% start(Depth);
% % ��������ͷ��ȡ����
% trigger(Depth);
% % �������ȡ�������ݣ�������������ts_depth��metaData_Depth
% [imgDepth, ~, ~] = getdata(Depth);
% 
% % ��kinect��ȶ�����ȡ��������
% ptCloud = pcfromkinect(Depth,imgDepth);
% % ֹͣkinect���
% stop(Depth);
% pcwrite(ptCloud,'test2.ply','Encoding','binary');

%% ��ʼ��������ʾ��
% ���ô�ֱ��Ϊy�ᣬ��ֱ�᷽��down

% player = pcplayer(ptCloud.XLimits, ptCloud.YLimits, ptCloud.ZLimits,...
%     'VerticalAxis', 'y', 'VerticalAxisDir', 'down');
% 
% % ����������
% xlabel(player.Axes, 'X (m)');
% ylabel(player.Axes, 'Y (m)');
% zlabel(player.Axes, 'Z (m)');
% % ��ʾ��������
% view(player, ptCloud);

%% �����ļ���ȡ
ptCloud = pcread('test2.ply');

%% ����Ԥ����
% % ���ָ���Ȥ����(m)��ֻ���˷�Χ�ڵ���
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

% ɾ����Ⱥ�㣬NumNeighbors������ڵ�������Threshold����Ⱥ����ֵ
[no_noise,inlierIndices,outlierIndices] = pcdenoise(patch,...
    'NumNeighbors', 40,'Threshold', 0.05);

% �²������֣�Ϊ�˼���ʹ�ã�����ע�ͣ�
percentage = 0.4;
no_noise = pcdownsample(no_noise, 'random',percentage);

%% �ָ��ƽ���ƽ���ϵ�����
% �ҵ�����ƽ�沢�Ƴ�����ƽ��㡣ʹ��RANSAC�㷨����ƥ�����ƽ�档
% ƽ��ķ��߷���Ӧ������ y ������ָ�����б����ֵ�����ĵ����
% ����ϵĵ���ƽ���2�������ڡ�

maxDistance = 0.02; % in meters
referenceVector = [0, 1, 0];
[~, Ground, outliers] = pcfitplane(no_noise, maxDistance, referenceVector);


% ѡ�����ڵ�ƽ��һ���ֵĵ㡣
pcWithoutGround = select(no_noise, outliers);

%% ����Ȥ���򻮷�
% ��ȥ�����ʶ��󣬼���x=[-0.5, 0.5] y=[-0.5 0.5] z=[1, 2]��Χ�ڵĵ�
roi = [-1 0.8 -1 1 1 2];
Maybe_target_index = findPointsInROI(pcWithoutGround,roi);
Maybe_target = select(pcWithoutGround, Maybe_target_index);
%% ����ŷ�Ͼ�����зָ�
% �趨��Сŷ�Ͼ��룬�ָ����ȡ��������е��������ĵ���Ŀ��
minDistance = 0.03;
% ����ŷ�Ͼ�����зָ�
[labels,~] = pcsegdist(Maybe_target,minDistance);

% ��labels��Ԫ�س��ִ���������ߴ������ϵ��µݼ����ұ߶�Ӧ�ı�ǩ
k=sort(labels');
w=diff(find([1 diff(k)==1 1]));
LabelsSeq = sortrows([w',unique(k)'], 'descend');

% �ָ����г��������������������
target_index_1 = find(labels==LabelsSeq(1,2));
target_1 = select(Maybe_target, target_index_1);

target_index_2 = find(labels==LabelsSeq(2,2));
target_2 = select(Maybe_target, target_index_2);
% pcshow(target);

%% ����OriBoundingBox
cornerPoints_1 = calc_OriBoundingBox(double(target_1.Location));
boxLine_1 = OBB_box_line(cornerPoints_1);

cornerPoints_2 = calc_OriBoundingBox(double(target_2.Location));
boxLine_2 = OBB_box_line(cornerPoints_2);

%% ��ɫ���
% �������ڱ�ǵ����ɫ��
colors = [0 0 1; ...  % ��ɫΪδ��ǵĵ�(0 0 1); ָ��Ϊ[R��G��B]
          0 1 0; ...  % ��ɫ��Ϊ����ƽ���(0 1 0)
          1 0 0; ...  % ��ɫ��Ϊ����1
          1 0 1; ...  % ��ɫ��Ϊ����2
          0 0 0];     % ��ɫ��Ϊ���Ҹ�����(0 0 0)
      
blueIdx  = 0; % ���������������ɫ��
greenIdx = 1; % ��ɫ��ǩ�����棩
redIdx   = 2; % ��ɫ��ǩ������1��
purpleIdx = 3; % ��ɫ��ǩ������2��
blackIdx = 4; % ��ɫ��ǩ���߿�
% ����ɫ��ǩ���ӵ������е�ÿ���㡣ʹ����ɫ��ʾ����ƽ��ͺ�ɫ������
labelSize   = [no_noise.Count, 1];
colorLabels = zeros(labelSize, 'single');

boxLineLabels_1 = blackIdx*ones([boxLine_1.Count, 1], 'single');
boxLineLabels_2 = blackIdx*ones([boxLine_2.Count, 1], 'single');


% �����ƽ��㡣
colorLabels(Ground) = greenIdx;
% ���ݱ�ǩ������Ŀ������Ⱦɫ(�������ݵ�no_noise��
colorLabels((outliers(Maybe_target_index(target_index_1)))) = redIdx;
colorLabels((outliers(Maybe_target_index(target_index_2)))) = purpleIdx;


%% �����ʾ
% �����б�ǵĵ���Ƶ����Ʋ������С�ʹ��ǰ�����õ�������ɫ��ǩ��
player_3 = pcplayer(no_noise.XLimits, no_noise.YLimits, no_noise.ZLimits,...
    'VerticalAxis', 'y', 'VerticalAxisDir', 'down');
colormap(player_3.Axes, colors)
% points1(inPlanePointIndices, :) = [];
% ֱ�ӽ��߿���ƺͱ�ǩ���ں���һ����ʾ
view(player_3, [no_noise.Location;boxLine_1.Location;boxLine_2.Location],...
    [colorLabels;boxLineLabels_1;boxLineLabels_2]);
title(player_3.Axes, 'Segmented Point Cloud');
