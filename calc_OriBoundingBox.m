% @author: Svenja (st100333@stud.uni-stuttgart.de)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% ��ϸ����������ݵ���СOBB�߽��ǵ�
%
% Input: 
%           �������ݵ�x,y,z����
%           �����ʽ��n*3����
% Output: 
%           ���ο�8�ǵ�x,y,z����
%           �����ʽ��8*3 ����
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function cornerpoints = calc_OriBoundingBox(data)
[n,dim] = size(data);

% ����������ݵ�͹�����õ���Χ���Ƶ���С������
% ����Ȼ����Ҳ�����������Ƕ��ڴ󲿷����͹�����Ը��õرƽ���С�߽��
convH = convhull(data(:,1),data(:,2),data(:,3));

% convhull����������ֻ��Ҫ�õ�͹���ϵĵ�����ˣ��൱�ڰ�����������
convH_points = data(convH(:),:);

% ���Э�������
nK = length(convH_points(:,1));
C = [convH_points(:,1)- sum(convH_points(:,1))/nK,convH_points(:,2)-sum(convH_points(:,2))/nK,convH_points(:,3)-sum(convH_points(:,3))/nK];
cov = C'*C; 

% ��������ֵ�ֽ�õ����ݾ�������ɷ�
[U,V,D] = svd(cov);

% ����ϵ�任����V�����������ųɵ�����ϵ
I = [1 0 0; 0 1 0; 0 0 1];
B_traf = U'\I;
data_traf = data;
for i = 1:n
    data_traf(i,:) = data(i,:)*B_traf;
end
% ����任����ϵ��ĵ������������߷���ı߿�
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

% ������ϵ�任��ԭʼ�ĵѿ�������ϵ
for j = 1:8
    cornerpoints(j,:) = cornerpoints(j,:)*B_traf';
end
end
