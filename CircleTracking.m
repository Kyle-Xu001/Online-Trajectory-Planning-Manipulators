clc;clear;

% 定义圆周运动目标点轨迹
theta_ = 0:0.001:2*pi;
x = 860 - 350*cos(theta_);
y = 350*sin(theta_);
z = 900+zeros(1,size(theta_,2));

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
robot.teach

% 定义轴空间的初始姿态，以及目标点位置
q1 = [0 -1.57 0 0 1.57 0];
tf = robot.fkine(q1);
p1 = [  tf.t(1)  tf.t(2)  tf.t(3)    0     0     0];
p_target = [    760       0    700      0      0      0;
                510       0    700      0      0      0;
                560   180.3    700      0      0      0;
                610   244.9    700      0      0      0;
                660   287.2    700      0      0      0;
                710   316.2    700      0      0      0;
                760   335.4    700      0      0      0;
                810   346.4    700      0      0      0;
                860   350.0    700      0      0      0;
                910   346.4    700      0      0      0;
                960   335.4    700      0      0      0;
               1010   316.2    700      0      0      0;
               1060   287.2    700      0      0      0;
               1110   244.9    700      0      0      0;
               1160   180.2    700      0      0      0;       
               1210       0    700      0      0      0;       
               1160  -180.2    700      0      0      0;
               1110  -244.9    700      0      0      0;
               1060  -287.2    700      0      0      0;
               1010  -316.2    700      0      0      0;
                960  -335.4    700      0      0      0;
                910  -346.4    700      0      0      0;
                860  -350.0    700      0      0      0;
                810  -346.4    700      0      0      0;
                760  -335.4    700      0      0      0;
                710  -316.2    700      0      0      0;
                660  -287.2    700      0      0      0;
                610  -244.9    700      0      0      0;
                560  -180.2    700      0      0      0;
                510       0    700      0      0      0];

            
num = 6500; % 迭代次数（即运动总时长）
lambda = 1e-6;% 误差系数（用于建立雅克比矩阵的伪逆）
k = 15; % 比例系数
n = 1;

% 定义机器人关节配置参数
j_min = -100;
j_max =  100;
a_max =   20;
a_min =  -20;
v_max =    5;
v_min =   -5;

ddq_ = zeros(num, axis);
dq_  = zeros(num, axis);
q_   = zeros(num, axis);
p_   = zeros(num,    6);
q_(1,:) = q1;
p_(1,1:3) = [p1(1), p1(2), p1(3)];
p2 = zeros(1,6);

for j = 2:num
    % 圆形轨迹绘制
    % 离散圆周轨迹点跟踪
%     p2 = p_target(n,:);

    
    % 连续圆周轨迹目标点跟踪
    if j <= size(x,2)
        p2(1) = x(j);
        p2(2) = y(j);
        p2(3) = z(j);
    end
    
    J = robot.jacob0(q1);
    J_T = transpose(J);
    J_inverse = J_T*(J*J_T+lambda)^(-1);
    delta_p = transpose(p2)-transpose(p1)
    dq = transpose(J_inverse*(k*delta_p));
    
%     % 离散点跟踪时请打开！
%     if abs(delta_p(1))<=30 && abs(delta_p(2))<=30 && abs(delta_p(3))<=30 && n~= 30
%        n = n + 1;
%     end

    % 最大/最小速度限制，等比例缩小至范围内；
    for i = 1:axis
        if dq(i) > v_max || dq(i) < v_min
            delta = max(abs(dq/v_max));
        else
            delta = 1;
        end
    end
    dq = dq/delta;
    
    % 最大/最小加速度限制，当超过加速度限制时，速度按每个周期所能达到的极限加速度进行变化；
    for i = 1:axis
        if (dq(i) - dq_(j-1,i))/0.001 > a_max
            dq(i) = dq_(j-1,i) + a_max * 0.001;
        elseif (dq(i) - dq_(j-1,i))/0.001 < a_min
            dq(i) = dq_(j-1,i) + a_min * 0.001;
        end
    end
    
    % 更新当前点笛卡尔坐标位置及轴空间坐标位置
    q1 = q1 + dq * 0.001;
    p = robot.fkine(q1);
    p1(1) = p.t(1);
    p1(2) = p.t(2);
    p1(3) = p.t(3);
    
    % 记录当前点笛卡尔坐标位置及轴空间坐标位置
    q_(j,:) = q1;
    dq_(j,:) = dq;
    ddq_(j,:) = (dq_(j,:) - dq_(j-1,:))/0.001;
    p_(j  ,1:3) = [p1(1),p1(2),p1(3)];
    p_(j-1,4:6) = [delta_p(1),delta_p(2),delta_p(3)];
end

figure(1)
subplot(6,1,1)
plot(0.001*(1:num),dq_(:,1));xlabel('时间/s');ylabel('速度/rad·s^{-1}');
subplot(6,1,2)
plot(0.001*(1:num),dq_(:,2));xlabel('时间/s');ylabel('速度/rad·s^{-1}');
subplot(6,1,3)
plot(0.001*(1:num),dq_(:,3));xlabel('时间/s');ylabel('速度/rad·s^{-1}');
subplot(6,1,4)
plot(0.001*(1:num),dq_(:,4));xlabel('时间/s');ylabel('速度/rad·s^{-1}');
subplot(6,1,5)
plot(0.001*(1:num),dq_(:,5));xlabel('时间/s');ylabel('速度/rad·s^{-1}');
subplot(6,1,6)
plot(0.001*(1:num),dq_(:,6));xlabel('时间/s');ylabel('速度/rad·s^{-1}');

figure(2)
subplot(6,1,1)
plot(0.001*(1:num),q_(:,1));xlabel('时间/s');ylabel('位置/rad');
subplot(6,1,2)
plot(0.001*(1:num),q_(:,2));xlabel('时间/s');ylabel('位置/rad');
subplot(6,1,3)
plot(0.001*(1:num),q_(:,3));xlabel('时间/s');ylabel('位置/rad');
subplot(6,1,4)
plot(0.001*(1:num),q_(:,4));xlabel('时间/s');ylabel('位置/rad');
subplot(6,1,5)
plot(0.001*(1:num),q_(:,5));xlabel('时间/s');ylabel('位置/rad');
subplot(6,1,6)
plot(0.001*(1:num),q_(:,6));xlabel('时间/s');ylabel('位置/rad');

figure(3)
subplot(6,1,1)
plot(0.001*(1:num),ddq_(:,1));xlabel('时间/s');ylabel('加速度/rad·s^{-2}');
subplot(6,1,2)
plot(0.001*(1:num),ddq_(:,2));xlabel('时间/s');ylabel('加速度/rad·s^{-2}');
subplot(6,1,3)
plot(0.001*(1:num),ddq_(:,3));xlabel('时间/s');ylabel('加速度/rad·s^{-2}');
subplot(6,1,4)
plot(0.001*(1:num),ddq_(:,4));xlabel('时间/s');ylabel('加速度/rad·s^{-2}');
subplot(6,1,5)
plot(0.001*(1:num),ddq_(:,5));xlabel('时间/s');ylabel('加速度/rad·s^{-2}');
subplot(6,1,6)
plot(0.001*(1:num),ddq_(:,6));xlabel('时间/s');ylabel('加速度/rad·s^{-2}');

figure(4)
subplot(6,1,1)
plot(0.001*(1:num),p_(:,1));xlabel('时间/s');ylabel('位置/mm');
subplot(6,1,2)
plot(0.001*(1:num),p_(:,2));xlabel('时间/s');ylabel('位置/mm');
subplot(6,1,3)
plot(0.001*(1:num),p_(:,3));xlabel('时间/s');ylabel('位置/mm');
subplot(6,1,4)
plot(0.001*(1:num),p_(:,4));xlabel('时间/s');ylabel('X位置差/mm');
subplot(6,1,5)
plot(0.001*(1:num),p_(:,5));xlabel('时间/s');ylabel('Y位置差/mm');
subplot(6,1,6)
plot(0.001*(1:num),p_(:,6));xlabel('时间/s');ylabel('Z位置差/mm');

figure(5)
plot3(p_(:,1),p_(:,2),p_(:,3),x,y,z)
xlabel('x/mm');ylabel('y/mm');zlabel('z/mm')
legend('末端执行器轨迹','目标点轨迹')
grid on


figure(6)
Tc = robot.fkine(q_);
Tjtraj = transl(Tc);
plot2(Tjtraj,'r')
robot.plot(q_(1:20:end,:))