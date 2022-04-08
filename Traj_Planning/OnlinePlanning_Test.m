%%
clc;clear;

% Call the robot initialization function
robot = robotInit();
axis = 6;

% Define the initial configuration and Start Point
q1 = [0 -1.57 0 0 1.57 0];
tf = robot.fkine(q1);
p1 = [  tf.t(1)  tf.t(2)  tf.t(3)    0     0     0];


num = 6000; % Iteration Times（Moving Time）
lambda = 1e-6; % Error Coefficient（used for pesudo-inverse of Jacobian Matrix）
k = 10; % Partial Coefficient
T_d = 10;
n = 1;

% Define the Initial State
state_init.q = q1;
state_init.dq = [0 0 0 0 0 0];
state_init.ddq = [0 0 0 0 0 0];
state_init.dddq = [0 0 0 0 0 0];
state_end.q = [0 0 0 0 0 0];
state_end.dq = [0 0 0 0 0 0];
state_end.ddq = [0 0 0 0 0 0];
state_end.dddq = [0 0 0 0 0 0];


% Declare the States for all Iterations
q_List = zeros(num, axis);
dq_List = zeros(num, axis);
ddq_List = zeros(num, axis);
dddq_List = zeros(num, axis);
p_List   = zeros(num,    6);

q_List(1,:) = q1;
p_List(1,1:3) = [p1(1), p1(2), p1(3)];

p2 = zeros(1,6);
state.q = zeros(10,axis);
state.dq = zeros(10,axis);
state.ddq = zeros(10,axis);
state.dddq = zeros(10,axis);

% Define the Goal Configuration
q2 = [0.523 -1.047 0.35 0.523 0.523 0];


% Start Robot Planning
for j = 1:num
    % Insert a goal updation
    if j == 500
        q2 = [-1.57 -1.57 0 0 1.57 0];
    end
    
    % Update current Cartesian Coordinates
    % Update Configuration Coordinates
    state_end.q =q2;
    
    for i = 1:axis
        % Copy the current states and goal states
        state_init_.q = state_init.q(i);
        state_init_.dq = state_init.dq(i);
        state_init_.ddq = state_init.ddq(i);
        state_init_.dddq = state_init.dddq(i);
        state_end_.q = state_end.q(i);
        state_end_.dq = state_end.dq(i);
        state_end_.ddq = state_end.ddq(i);
        state_end_.dddq = state_end.dddq(i);
        
        % Call OnlinePlanning funtion to calculate state for each interval
        state_ = OnlinePlanning(state_init_,state_end_);
        state.q(:,i) = state_.q;
        state.dq(:,i) = state_.dq;
        state.ddq(:,i) = state_.ddq;
        state.dddq(:,i) = state_.dddq;
    end
    
    state_init.q = state.q(end,:);
    state_init.q
    state_init.dq = state.dq(end,:);
    state_init.ddq = state.ddq(end,:);
    state_init.dddq = state.dddq(end,:);
    
    p = robot.fkine(state_init.q);
    p1(1) = p.t(1);
    p1(2) = p.t(2);
    p1(3) = p.t(3);
    
    % Record current Cartesian Coordinates and Configuration Coordinates 
    q_List(j+1,:) = state_init.q;
    dq_List(j+1,:) = state_init.dq;
    ddq_List(j+1,:) = state_init.ddq;
    p_List(j+1  ,1:3) = [p1(1),p1(2),p1(3)];
end

figure(1)
subplot(6,1,1)
plot(0.001*(1:num+1),dq_List(:,1));xlabel('Time/s');ylabel('Angular Speed/rad·s^{-1}');
subplot(6,1,2)
plot(0.001*(1:num+1),dq_List(:,2));xlabel('Time/s');ylabel('Angular Speed/rad·s^{-1}');
subplot(6,1,3)
plot(0.001*(1:num+1),dq_List(:,3));xlabel('Time/s');ylabel('Angular Speed/rad·s^{-1}');
subplot(6,1,4)
plot(0.001*(1:num+1),dq_List(:,4));xlabel('Time/s');ylabel('Angular Speed/rad·s^{-1}');
subplot(6,1,5)
plot(0.001*(1:num+1),dq_List(:,5));xlabel('Time/s');ylabel('Angular Speed/rad·s^{-1}');
subplot(6,1,6)
plot(0.001*(1:num+1),dq_List(:,6));xlabel('Time/s');ylabel('Angular Speed/rad·s^{-1}');

figure(2)
subplot(6,1,1)
plot(0.001*(1:num+1),q_List(:,1));xlabel('Time/s');ylabel('Angle/rad');
subplot(6,1,2)
plot(0.001*(1:num+1),q_List(:,2));xlabel('Time/s');ylabel('Angle/rad');
subplot(6,1,3)
plot(0.001*(1:num+1),q_List(:,3));xlabel('Time/s');ylabel('Angle/rad');
subplot(6,1,4)
plot(0.001*(1:num+1),q_List(:,4));xlabel('Time/s');ylabel('Angle/rad');
subplot(6,1,5)
plot(0.001*(1:num+1),q_List(:,5));xlabel('Time/s');ylabel('Angle/rad');
subplot(6,1,6)
plot(0.001*(1:num+1),q_List(:,6));xlabel('Time/s');ylabel('Angle/rad');

figure(3)
subplot(6,1,1)
plot(0.001*(1:num+1),ddq_List(:,1));xlabel('Time/s');ylabel('Augular Acceleration/rad·s^{-2}');
subplot(6,1,2)
plot(0.001*(1:num+1),ddq_List(:,2));xlabel('Time/s');ylabel('Augular Acceleration/rad·s^{-2}');
subplot(6,1,3)
plot(0.001*(1:num+1),ddq_List(:,3));xlabel('Time/s');ylabel('Augular Acceleration/rad·s^{-2}');
subplot(6,1,4)
plot(0.001*(1:num+1),ddq_List(:,4));xlabel('Time/s');ylabel('Augular Acceleration/rad·s^{-2}');
subplot(6,1,5)
plot(0.001*(1:num+1),ddq_List(:,5));xlabel('Time/s');ylabel('Augular Acceleration/rad·s^{-2}');
subplot(6,1,6)
plot(0.001*(1:num+1),ddq_List(:,6));xlabel('Time/s');ylabel('Augular Acceleration/rad·s^{-2}');

figure(4)
subplot(3,1,1)
plot(0.001*(1:num+1),p_List(:,1));xlabel('Time/s');ylabel('Position/mm');
subplot(3,1,2)
plot(0.001*(1:num+1),p_List(:,2));xlabel('Time/s');ylabel('Position/mm');
subplot(3,1,3)
plot(0.001*(1:num+1),p_List(:,3));xlabel('Time/s');ylabel('Position/mm');


figure(5)
plot3(p_List(:,1),p_List(:,2),p_List(:,3))
xlabel('x/mm');ylabel('y/mm');zlabel('z/mm')
legend('Trajectory of End Effector')
grid on


figure(6)
Tc = robot.fkine(q_List);
Tjtraj = transl(Tc);
plot2(Tjtraj,'r')
robot.plot(q_List(1:20:end,:))