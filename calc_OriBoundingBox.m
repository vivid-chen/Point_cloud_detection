% @author: Svenja (st100333@stud.uni-stuttgart.de)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 拟合给定点云数据的最小OBB边界框角点
%
% Input: 
%           点云数据点x,y,z坐标
%           输入格式：n*3矩阵
% Output: 
%           矩形框8角点x,y,z坐标
%           输出格式：8*3 矩阵
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function cornerpoints = calc_OriBoundingBox(data)
[n,dim] = size(data);

% 计算点云数据的凸包，得到包围点云的最小多面体
% （虽然不算也可以做，但是对于大部分情况凸包可以更好地逼近最小边界框
convH = convhull(data(:,1),data(:,2),data(:,3));

% convhull函数求解过后，只需要得到凸包上的点就行了，相当于包裹物体的外壳
convH_points = data(convH(:),:);

% 求解协方差矩阵
nK = length(convH_points(:,1));
C = [convH_points(:,1)- sum(convH_points(:,1))/nK,convH_points(:,2)-sum(convH_points(:,2))/nK,convH_points(:,3)-sum(convH_points(:,3))/nK];
cov = C'*C; 

% 利用奇异值分解得到数据矩阵的主成分
[U,V,D] = svd(cov);

% 坐标系变换成由V的特征向量张成的坐标系
I = [1 0 0; 0 1 0; 0 0 1];
B_traf = U'\I;
data_traf = data;
for i = 1:n
    data_traf(i,:) = data(i,:)*B_traf;
end
% 计算变换坐标系后的点云数据沿轴线方向的边框
cornerpoints = zeros(8,3);
x = data_traf(:,1);
y = data_traf(:,2);
z = data_traf(:,3);
cornerpoints(1,:) = [min(x), min(y), min(z)];
cornerpoints(2,:) = [max(x), min(y), min(z)];
cornerpoints(3,:) = [max(x), max(y), min(z)];
cornerpoints(4,:) = [min(x), max(y), min(z)];
cornerpoints(5,:) = [min(x), max(y), max(z)];
cornerpoints(6,:) = [max(x), max(y), max(z)];
cornerpoints(7,:) = [max(x), min(y), max(z)];
cornerpoints(8,:) = [min(x), min(y), max(z)];

% 将坐标系变换回原始的笛卡尔坐标系
for j = 1:8
    cornerpoints(j,:) = cornerpoints(j,:)*B_traf';
end
end
