clear;
clc;                                                                                                                                  

% ========== 机器人模型定义 ==========

L(1) = Link('revolute','d',0,'a',0,'alpha',0,'modified');
L(2) = Link('revolute','d',0,'a',0,'alpha',pi/2,'modified');
L(3) = Link('revolute','d',0,'a',0.09,'alpha',0, 'offset',pi-atan(35.5/82.7),'modified');
L(4) = Link('revolute','d',0,'a',0.09,'alpha',0, 'offset',atan(35.5/82.7),'modified');
Four_dof_mod = SerialLink(L, 'name', '四轴机械臂');
Four_dof_mod.base = transl(0, 0, 0.08);
Four_dof_mod.tool = transl(0.011, 0, 0);
Four_dof_mod.teach;



% ========== 工作空间可视化 ==========
% 蒙特卡洛法参数设置
num_points = 3000;  % 采样点数量
workspace_points = zeros(num_points, 3);  % 预分配内存

% 生成随机关节角并计算末端位置
for i = 1:num_points
    % 生成随机关节角度（在限制范围内）
    q1 = L(1).qlim(1) + diff(L(1).qlim)*rand();
    q2 = L(2).qlim(1) + diff(L(2).qlim)*rand();
    q3 = L(3).qlim(1) + diff(L(3).qlim)*rand();
    q4 = L(4).qlim(1) + diff(L(4).qlim)*rand();
    
    % 计算正运动学
    T = Four_dof_mod.fkine([q1, q2, q3, q4]);
    
    % 提取末端位置坐标
    workspace_points(i,:) = T.t(1:3);
end

% 可视化工作空间
figure('Name', '工作空间可视化')
scatter3(workspace_points(:,1), workspace_points(:,2), workspace_points(:,3),...
         5, 'filled', 'MarkerFaceAlpha', 0.4)
xlabel('X轴 (m)'); ylabel('Y轴 (m)'); zlabel('Z轴 (m)')
title('机械臂工作空间点云图')
axis equal
grid on

% 保持机器人模型显示
hold on   
Four_dof_mod.plot([0, 0, 0, 0], 'nobase', 'notiles', 'noarrow')
hold off                                                                        
