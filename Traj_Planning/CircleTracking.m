clc;clear;

% Define the Target Trajectory
theta_ = 0:0.001:2*pi;
x = 860 - 350*cos(theta_);
y = 350*sin(theta_);
z = 900+zeros(1,size(theta_,2));

% Call the robot initialization function
robot = robotInit();
axis = 6;

% Define the initial configuration and Target Point
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

            
num = 6500; % Iteration Times（Moving Time）
lambda = 1e-6; % Error Coefficient（used for pesudo-inverse of Jacobian Matrix）
k = 15; % Partial Coefficient
n = 1;

% Define the joint configuration limitations
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
    % Discrete Trajectory Point Following
%     p2 = p_target(n,:);

    
    % Continuous Trajectory Point Following
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
    
%     % Please Open when following Discrete Points
%     if abs(delta_p(1))<=30 && abs(delta_p(2))<=30 && abs(delta_p(3))<=30 && n~= 30
%        n = n + 1;
%     end

    % Constraints on velocity, to resize the sped into limitations
    for i = 1:axis
        if dq(i) > v_max || dq(i) < v_min
            delta = max(abs(dq/v_max));
        else
            delta = 1;
        end
    end
    dq = dq/delta;
    
    % Constraints on acceleration, choose the maximum acceleration
    for i = 1:axis
        if (dq(i) - dq_(j-1,i))/0.001 > a_max
            dq(i) = dq_(j-1,i) + a_max * 0.001;
        elseif (dq(i) - dq_(j-1,i))/0.001 < a_min
            dq(i) = dq_(j-1,i) + a_min * 0.001;
        end
    end
    
    % Update current Cartesian Coordinates
    % Update Configuration Coordinates
    q1 = q1 + dq * 0.001;
    p = robot.fkine(q1);
    p1(1) = p.t(1);
    p1(2) = p.t(2);
    p1(3) = p.t(3);
    
    % Record current Cartesian Coordinates and Configuration Coordinates 
    q_(j,:) = q1;
    dq_(j,:) = dq;
    ddq_(j,:) = (dq_(j,:) - dq_(j-1,:))/0.001;
    p_(j  ,1:3) = [p1(1),p1(2),p1(3)];
    p_(j-1,4:6) = [delta_p(1),delta_p(2),delta_p(3)];
end

figure(1)
subplot(6,1,1)
plot(0.001*(1:num),dq_(:,1));xlabel('Time/s');ylabel('Speed/rad·s^{-1}');
subplot(6,1,2)
plot(0.001*(1:num),dq_(:,2));xlabel('Time/s');ylabel('Speed/rad·s^{-1}');
subplot(6,1,3)
plot(0.001*(1:num),dq_(:,3));xlabel('Time/s');ylabel('Speed/rad·s^{-1}');
subplot(6,1,4)
plot(0.001*(1:num),dq_(:,4));xlabel('Time/s');ylabel('Speed/rad·s^{-1}');
subplot(6,1,5)
plot(0.001*(1:num),dq_(:,5));xlabel('Time/s');ylabel('Speed/rad·s^{-1}');
subplot(6,1,6)
plot(0.001*(1:num),dq_(:,6));xlabel('Time/s');ylabel('Speed/rad·s^{-1}');

figure(2)
subplot(6,1,1)
plot(0.001*(1:num),q_(:,1));xlabel('Time/s');ylabel('Angle/mm');
subplot(6,1,2)
plot(0.001*(1:num),q_(:,2));xlabel('Time/s');ylabel('Angle/mm');
subplot(6,1,3)
plot(0.001*(1:num),q_(:,3));xlabel('Time/s');ylabel('Angle/mm');
subplot(6,1,4)
plot(0.001*(1:num),q_(:,4));xlabel('Time/s');ylabel('Angle/mm');
subplot(6,1,5)
plot(0.001*(1:num),q_(:,5));xlabel('Time/s');ylabel('Angle/mm');
subplot(6,1,6)
plot(0.001*(1:num),q_(:,6));xlabel('Time/s');ylabel('Angle/mm');

figure(3)
subplot(6,1,1)
plot(0.001*(1:num),ddq_(:,1));xlabel('Time/s');ylabel('Acceleration/rad·s^{-2}');
subplot(6,1,2)
plot(0.001*(1:num),ddq_(:,2));xlabel('Time/s');ylabel('Acceleration/rad·s^{-2}');
subplot(6,1,3)
plot(0.001*(1:num),ddq_(:,3));xlabel('Time/s');ylabel('Acceleration/rad·s^{-2}');
subplot(6,1,4)
plot(0.001*(1:num),ddq_(:,4));xlabel('Time/s');ylabel('Acceleration/rad·s^{-2}');
subplot(6,1,5)
plot(0.001*(1:num),ddq_(:,5));xlabel('Time/s');ylabel('Acceleration/rad·s^{-2}');
subplot(6,1,6)
plot(0.001*(1:num),ddq_(:,6));xlabel('Time/s');ylabel('Acceleration/rad·s^{-2}');

figure(4)
subplot(6,1,1)
plot(0.001*(1:num),p_(:,1));xlabel('Time/s');ylabel('Position/mm');
subplot(6,1,2)
plot(0.001*(1:num),p_(:,2));xlabel('Time/s');ylabel('Position/mm');
subplot(6,1,3)
plot(0.001*(1:num),p_(:,3));xlabel('Time/s');ylabel('Position/mm');
subplot(6,1,4)
plot(0.001*(1:num),p_(:,4));xlabel('Time/s');ylabel('X Error/mm');
subplot(6,1,5)
plot(0.001*(1:num),p_(:,5));xlabel('Time/s');ylabel('Y Error/mm');
subplot(6,1,6)
plot(0.001*(1:num),p_(:,6));xlabel('Time/s');ylabel('Z Error/mm');

figure(5)
plot3(p_(:,1),p_(:,2),p_(:,3),x,y,z)
xlabel('x/mm');ylabel('y/mm');zlabel('z/mm')
legend('End Effector Trajectory','Goal Trajectory')
grid on


figure(6)
Tc = robot.fkine(q_);
Tjtraj = transl(Tc);
plot2(Tjtraj,'r')
robot.plot(q_(1:20:end,:))