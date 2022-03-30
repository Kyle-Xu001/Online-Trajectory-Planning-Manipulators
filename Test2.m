%%
clc;clear;


% DH参数定义
d     = [ 520,      0,   0,    660,     0,    115]; 
a     = [   0,    100, 680,     80,     0,      0];
alpha = [   0,  -pi/2,   0,  -pi/2,  pi/2,  -pi/2];

% 确定机器人轴数
axis = size(d,2);

% 建立机器人模型
%         theta    d        a        alpha
L = Link([  0     d(1)     a(1)     alpha(1)],'modified');
for i = 2:axis
    eval(['L',num2str(i),'=Link([0     d(i)       a(i)       alpha(i)],"modified");'])
    L = eval(['L + L',num2str(i)]);
end
robot=SerialLink(L,'name','XB12');

% 定义轴空间的初始姿态，以及目标点位置
q1 = [0 -1.57 0 0 1.57 0];
tf = robot.fkine(q1);
p1 = [  tf.t(1)  tf.t(2)  tf.t(3)    0     0     0];


num = 6000; % 迭代次数（即运动总时长）
lambda = 1e-6;% 误差系数（用于建立雅克比矩阵的伪逆）
k = 10; % 比例系数
T_d = 10;
n = 1;


state_init.q = q1;
state_init.dq = [0 0 0 0 0 0];
state_init.ddq = [0 0 0 0 0 0];
state_init.dddq = [0 0 0 0 0 0];
state_end.q = [0 0 0 0 0 0];
state_end.dq = [0 0 0 0 0 0];
state_end.ddq = [0 0 0 0 0 0];
state_end.dddq = [0 0 0 0 0 0];



q_ = zeros(num, axis);
dq_ = zeros(num, axis);
ddq_ = zeros(num, axis);
dddq_ = zeros(num, axis);
p_   = zeros(num,    6);
q_(1,:) = q1;
p_(1,1:3) = [p1(1), p1(2), p1(3)];
p2 = zeros(1,6);
state.q = zeros(10,axis);
state.dq = zeros(10,axis);
state.ddq = zeros(10,axis);
state.dddq = zeros(10,axis);

q2 = [0.523 -1.047 0.35 0.523 0.523 0];
for j = 1:num
        
    if j == 500
        q2 = [-1.57 -1.57 0 0 1.57 0];
    end
    % 更新当前点笛卡尔坐标位置及轴空间坐标位置
    q2
    state_end.q =q2;
    
    for i = 1:axis
        state_init_.q = state_init.q(i);
        state_init_.dq = state_init.dq(i);
        state_init_.ddq = state_init.ddq(i);
        state_init_.dddq = state_init.dddq(i);
        state_end_.q = state_end.q(i);
        state_end_.dq = state_end.dq(i);
        state_end_.ddq = state_end.ddq(i);
        state_end_.dddq = state_end.dddq(i);
        state_ = OnlinePlanning(state_init_,state_end_);
        state.q(:,i) = state_.q;
        state.dq(:,i) = state_.dq;
        state.ddq(:,i) = state_.ddq;
        state.dddq(:,i) = state_.dddq;
    end
    state_init.q = state.q(end,:);
    state_init.dq = state.dq(end,:);
    state_init.ddq = state.ddq(end,:);
    state_init.dddq = state.dddq(end,:);
    
    
    p = robot.fkine(state_init.q);
    p1(1) = p.t(1);
    p1(2) = p.t(2);
    p1(3) = p.t(3);
    
    % 记录当前点笛卡尔坐标位置及轴空间坐标位置
    q_(j+1,:) = state_init.q;
    dq_(j+1,:) = state_init.dq;
    ddq_(j+1,:) = state_init.ddq;
    p_(j+1  ,1:3) = [p1(1),p1(2),p1(3)];
end

figure(1)
subplot(6,1,1)
plot(0.001*(1:num+1),dq_(:,1));xlabel('时间/s');ylabel('速度/rad·s^{-1}');
subplot(6,1,2)
plot(0.001*(1:num+1),dq_(:,2));xlabel('时间/s');ylabel('速度/rad·s^{-1}');
subplot(6,1,3)
plot(0.001*(1:num+1),dq_(:,3));xlabel('时间/s');ylabel('速度/rad·s^{-1}');
subplot(6,1,4)
plot(0.001*(1:num+1),dq_(:,4));xlabel('时间/s');ylabel('速度/rad·s^{-1}');
subplot(6,1,5)
plot(0.001*(1:num+1),dq_(:,5));xlabel('时间/s');ylabel('速度/rad·s^{-1}');
subplot(6,1,6)
plot(0.001*(1:num+1),dq_(:,6));xlabel('时间/s');ylabel('速度/rad·s^{-1}');

figure(2)
subplot(6,1,1)
plot(0.001*(1:num+1),q_(:,1));xlabel('时间/s');ylabel('位置/rad');
subplot(6,1,2)
plot(0.001*(1:num+1),q_(:,2));xlabel('时间/s');ylabel('位置/rad');
subplot(6,1,3)
plot(0.001*(1:num+1),q_(:,3));xlabel('时间/s');ylabel('位置/rad');
subplot(6,1,4)
plot(0.001*(1:num+1),q_(:,4));xlabel('时间/s');ylabel('位置/rad');
subplot(6,1,5)
plot(0.001*(1:num+1),q_(:,5));xlabel('时间/s');ylabel('位置/rad');
subplot(6,1,6)
plot(0.001*(1:num+1),q_(:,6));xlabel('时间/s');ylabel('位置/rad');

figure(3)
subplot(6,1,1)
plot(0.001*(1:num+1),ddq_(:,1));xlabel('时间/s');ylabel('加速度/rad·s^{-2}');
subplot(6,1,2)
plot(0.001*(1:num+1),ddq_(:,2));xlabel('时间/s');ylabel('加速度/rad·s^{-2}');
subplot(6,1,3)
plot(0.001*(1:num+1),ddq_(:,3));xlabel('时间/s');ylabel('加速度/rad·s^{-2}');
subplot(6,1,4)
plot(0.001*(1:num+1),ddq_(:,4));xlabel('时间/s');ylabel('加速度/rad·s^{-2}');
subplot(6,1,5)
plot(0.001*(1:num+1),ddq_(:,5));xlabel('时间/s');ylabel('加速度/rad·s^{-2}');
subplot(6,1,6)
plot(0.001*(1:num+1),ddq_(:,6));xlabel('时间/s');ylabel('加速度/rad·s^{-2}');

figure(4)
subplot(3,1,1)
plot(0.001*(1:num+1),p_(:,1));xlabel('时间/s');ylabel('位置/mm');
subplot(3,1,2)
plot(0.001*(1:num+1),p_(:,2));xlabel('时间/s');ylabel('位置/mm');
subplot(3,1,3)
plot(0.001*(1:num+1),p_(:,3));xlabel('时间/s');ylabel('位置/mm');


figure(5)
plot3(p_(:,1),p_(:,2),p_(:,3))
xlabel('x/mm');ylabel('y/mm');zlabel('z/mm')
legend('末端执行器轨迹')
grid on


figure(6)
Tc = robot.fkine(q_);
Tjtraj = transl(Tc);
plot2(Tjtraj,'r')
robot.plot(q_(1:20:end,:))