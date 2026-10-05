clc
m = 0.0923;						% 摆杆的质量
M = 0.1945; 					% 滑块的质量
l = 0.1950; 					% 摆杆质心到转轴的距离
J = 0.0029; 					% 摆杆绕质心转动时的转动惯量
g = 9.8; 						% 重力加速度
R = 3.75; 						% 电枢电阻
r = 0.018; 						% 同步带轮的半径
K_m = 0.183; 					% 电磁转矩系数
K_e = 0.232; 					% 反电动势系数
I = 7.083e-06; 					% 同步带轮绕电机轴转动时的转动惯量
Q_eq = m*J+(J+m*l^2)*(M+I/r^2);
A_22 = -(K_m*K_e*(J+m*l^2))/(Q_eq*R*r^2);
A_23 = -(m^2*l^2*g)/Q_eq;
A_42 = (m*l*K_m*K_e)/(Q_eq*R*r^2);
A_43 = (m*g*l*(M+m+I/r^2))/Q_eq;
B_21 = (K_m*(J+m*l^2))/(Q_eq*R*r);
B_41 = -(m*l*K_m)/(Q_eq*R*r);
A = [0 1 0 0; 0 A_22/2 A_23 0; 0 0 0 1; 0 A_42/2 A_43 0];
B = [0; B_21/2; 0; B_41/2];
C = [1 0 0 0; 0 0 1 0]; 
D =[0 0]';
Ts = 0.020; 						% 采样间隔
t = 0:Ts:8;
u = zeros(size(t));
[G,H] = c2d(A,B,Ts); 			    % 将连续系统变为离散系统
x0 = [0; 0; 0.1745; 0]; 			% 设定系统的初始状态
Tc = ctrb(G,H);
if (rank(Tc)==4)
    fprintf('此系统是可控的！\n');
    Q = [7000 0 0 0; 0 0 0 0; 0 0 2000 0; 0 0 0 0];	 % Q矩阵
    R = 5;											     % R矩阵
    K = dlqr(G,H,Q,R); 			% 计算状态反馈系数（自行查看计算后的数值）
    G2 = G-H*K;
    y = dlsim(G2,H,C,D,u,x0);
    subplot(2,1,1)
    plot(t,y(:,1),'b','LineWidth',1.5);
    grid on
    xlabel('Time(s)');
    ylabel('Displacement(m)');
    subplot(2,1,2)
    plot(t,y(:,2),'b','LineWidth',1.5);
    grid on
    xlabel('Time(s)');
    ylabel('Angle(rad)');
end
K


% ===== 画出实际控制电压 =====
[y, states] = dlsim(G2, H, C, D, u, x0);

% 状态反馈控制量
% u = -Kx
u_control = -(K * states.').';

figure;
plot(t, u_control, 'LineWidth', 1.5);
yline(11.5, '--r');
yline(-11.5, '--r');

xlabel('Time (s)');
ylabel('Control voltage (V)');
title('LQR Control Input');
grid on;