function robot = robotInit()

% Define DH parameters
d     = [ 520,      0,   0,    660,     0,    115]; 
a     = [   0,    100, 680,     80,     0,      0];
alpha = [   0,  -pi/2,   0,  -pi/2,  pi/2,  -pi/2];

% Define the axis of the robot
axis = size(d,2);

% Build the Robot Model using modified DH parameters
%         theta    d        a        alpha
L = Link([  0     d(1)     a(1)     alpha(1)], 'modified');
for i = 2:axis
    eval(['L',num2str(i),'=Link([0 d(i) a(i) alpha(i)],"modified");'])
    L = eval(['L + L',num2str(i)]);
end

robot=SerialLink(L,'name','XB12');
% robot.teach