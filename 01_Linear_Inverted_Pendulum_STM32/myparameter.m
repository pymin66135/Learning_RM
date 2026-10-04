%% 

clc;
clear;
close all;

%% =========================
%  倒立摆系统参数
% =========================
m = 0.0923;                     % 摆杆质量
M = 0.1945;                     % 滑块质量
l = 0.1950;                     % 摆杆质心到转轴距离
J = 0.0029;                     % 摆杆绕质心转动惯量
g = 9.8;                        % 重力加速度

Ra = 3.75;                      % 电枢电阻
r = 0.018;                      % 同步带轮半径
K_m = 0.183;                    % 电磁转矩系数
K_e = 0.232;                    % 反电动势系数
I = 7.083e-06;                  % 同步带轮转动惯量


%% =========================
%  建立连续状态空间模型
% =========================
Q_eq = m*J + (J + m*l^2)*(M + I/r^2);

A_22 = -(K_m*K_e*(J+m*l^2)) / (Q_eq*Ra*r^2);
A_23 = -(m^2*l^2*g) / Q_eq;

A_42 = (m*l*K_m*K_e) / (Q_eq*Ra*r^2);
A_43 = (m*g*l*(M+m+I/r^2)) / Q_eq;

B_21 = (K_m*(J+m*l^2)) / (Q_eq*Ra*r);
B_41 = -(m*l*K_m) / (Q_eq*Ra*r);


A = [0 1 0 0;
     0 A_22/2 A_23 0;
     0 0 0 1;
     0 A_42/2 A_43 0];

B = [0;
     B_21/2;
     0;
     B_41/2];


%% =========================
%  离散化
% =========================
Ts = 0.020;                     % 采样周期 20ms
t = 0:Ts:8;

[G,H] = c2d(A,B,Ts);


%% =========================
%  初始状态
%
%  x = [位置;
%       速度;
%       角度;
%       角速度]
% =========================
x0 = [0;
      0;
      0.1745;       % 约10°
      0];


%% =========================
%  可控性判断
% =========================
Tc = ctrb(G,H);

if rank(Tc) ~= 4
    error('系统不可控！');
else
    fprintf('此系统是可控的！\n');
end


%% ============================================================
%  设置不同的 Q、R
% ============================================================

Q_set = {

    diag([1000, 0, 4000, 0]), ...      % 1 基准

    diag([5000, 0, 4000, 0]), ...      % 2 更重视位置

    diag([1000, 0, 12000, 0]), ...     % 3 更重视角度

    diag([1000, 0, 4000, 0]), ...      % 4 控制更加激进

    diag([1000, 0, 4000, 0])           % 5 控制更加保守
};


R_set = {
    0.1, ...       % 基准
    0.1, ...       % 增大位置权重
    0.1, ...       % 增大角度权重
    0.01, ...      % R变小：允许更大的控制输入
    1.0            % R变大：限制控制输入
};


name_set = {
    '基准 Q=[1000,0,4000,0], R=0.1'
    '位置权重大 Qx=5000'
    '角度权重大 Qtheta=12000'
    'R=0.01 激进控制'
    'R=1 保守控制'
};


%% ============================================================
%  仿真
% ============================================================

N_case = length(Q_set);

position_result = zeros(length(t), N_case);
angle_result    = zeros(length(t), N_case);
control_result  = zeros(length(t), N_case);


for i = 1:N_case

    Q_lqr = Q_set{i};
    R_lqr = R_set{i};

    % LQR反馈矩阵
    K = dlqr(G,H,Q_lqr,R_lqr);

    fprintf('\n---------------------------------\n');
    fprintf('Case %d\n',i);
    fprintf('R = %.4f\n',R_lqr);
    fprintf('K = \n');
    disp(K);


    %% 闭环系统
    %
    % x[k+1] = (G-HK)x[k]
    %
    G_cl = G - H*K;


    %% 仿真所有状态
    %
    % 没有外部输入，因此输入设为0
    %
    u_zero = zeros(length(t),1);

    C_state = eye(4);
    D_state = zeros(4,1);

    x = dlsim(G_cl,H,C_state,D_state,u_zero,x0);


    %% 保存结果
    position_result(:,i) = x(:,1);
    angle_result(:,i)    = x(:,3);


    %% 实际LQR控制输入
    %
    % u[k] = -K*x[k]
    %
    control_result(:,i) = -(x*K');

end


%% ============================================================
%  图1：滑块位置
% ============================================================

figure;

hold on;

for i = 1:N_case
    plot(t,position_result(:,i), ...
        'LineWidth',1.5);
end

grid on;
xlabel('Time (s)');
ylabel('Displacement (m)');
title('不同 Q、R 下的滑块位置响应');

legend(name_set,'Location','best');

hold off;



%% ============================================================
%  图2：摆杆角度
% ============================================================

figure;

hold on;

for i = 1:N_case
    plot(t,angle_result(:,i), ...
        'LineWidth',1.5);
end

grid on;
xlabel('Time (s)');
ylabel('Angle (rad)');
title('不同 Q、R 下的摆杆角度响应');

legend(name_set,'Location','best');

hold off;



%% ============================================================
%  图3：控制输入
% ============================================================

figure;

hold on;

for i = 1:N_case
    plot(t,control_result(:,i), ...
        'LineWidth',1.5);
end

grid on;
xlabel('Time (s)');
ylabel('Control input u');
title('不同 Q、R 下的控制输入');

legend(name_set,'Location','best');

hold off;