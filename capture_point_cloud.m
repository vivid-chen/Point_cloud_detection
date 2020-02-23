clc;
clear;

%创建点云对象
colorVid = imaq.VideoDevice('kinect',1);
depthVid = imaq.VideoDevice('kinect',2);

depthImage = depthVid();
ptCloud = pcfromkinect(depthVid, depthImage);


player = pcplayer(ptCloud.XLimits, ptCloud.YLimits, ptCloud.ZLimits,...
    'VerticalAxis', 'y', 'VerticalAxisDir', 'down');

xlabel(player.Axes, 'X (m)');
ylabel(player.Axes, 'Y (m)');
zlabel(player.Axes, 'Z (m)');

view(player, ptCloud);

%停止设备
stop([colorVid,depthVid]); 

release([colorVid,depthVid]);
% --------------------------------------------------------------------

% colorDevice = imaq.VideoDevice('kinect',1);
% depthDevice = imaq.VideoDevice('kinect',2);
% 
% %init
% colorDevice();
% depthDevice();
% 
% %one frame
% colorImage = colorDevice();
% depthImage = depthDevice();
% 
% ptCloud = pcfromkinect(depthDevice, depthImage, colorImage);
% % ptCloud = pcfromkinect(depthDevice, depthImage);
% 
% % Initialize a player to visualize 3-D point cloud data. The axis is
% % set appropriately to visualize the point cloud from Kinect.
% player = pcplayer(ptCloud.XLimits, ptCloud.YLimits, ptCloud.ZLimits,...
%               'VerticalAxis', 'y', 'VerticalAxisDir', 'down');
% 
% xlabel(player.Axes, 'X (m)');
% ylabel(player.Axes, 'Y (m)');
% zlabel(player.Axes, 'Z (m)');
% 
% % Acquire and view Kinect point cloud data.
% while isOpen(player)
%     colorImage = colorDevice();
% 	depthImage = depthDevice();
% 
% 	ptCloud = pcfromkinect(depthDevice, depthImage, colorImage);
%     % ptCloud = pcfromkinect(depthDevice, depthImage);
%     
% 	view(player, ptCloud);
% end