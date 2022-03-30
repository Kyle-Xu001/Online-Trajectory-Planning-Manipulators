clc;clear;

q_target = [2 1.5 1.5 2 1 0.5 2];
q_cur = [0 0 0 0 0 0 0];
init(q_cur);
len=100000;

q_cmd=zeros(len+1,7);
q_cmd(1,:) = q_cur;

for i= 1:len
    dq_des = (q_target - q_cur)/0.001;
    q_cur = apply(dq_des);
    q_cmd(i+1,:)=q_cur;
    if i ==30000
        q_target = [0.025 0.01 0.03 0.02 0.01 0.05 0.15];
    end
    
    if i == 50000
        q_target = [0.01 0.015 0.025 0.04 0.05 0.01 0.10];
    end        
end

dq_cmd = zeros(len,7);
for i = 2:len+1
    dq_cmd(i-1,:) = (q_cmd(i,:)-q_cmd(i-1,:))/0.001;
end

ddq_cmd = zeros(len-1,7);
for i = 2:len
    ddq_cmd(i-1,:) = (dq_cmd(i,:)-dq_cmd(i-1,:))/0.001;
end


for i = 1:7
    figure(1)
    subplot(7,1,i)
    plot((0:len)*0.001,q_cmd(:,i));
    xlabel('时间/s');ylabel('位置/rad');
end

for i = 1:7
    figure(2)
    subplot(7,1,i)
    plot((0:len-1)*0.001,dq_cmd(:,i));
    xlabel('时间/s');ylabel('速度/rad·s^{-1}');
end

for i = 1:7
    figure(3)
    subplot(7,1,i)
    plot((0:len-2)*0.001,ddq_cmd(:,i));
    xlabel('时间/s');ylabel('加速度/rad·s^{-2}');
end



% clear;clc;
% % input value
% jerk = 1500; % rad/(s^3)
% a_max = 100; % rad/(s^2)
% v_max = 5; % rad/s
% 
% if v_max < a_max ^2 / jerk
%     % 这表示极限加速过程中，未达到最大加速度便达到最大速度
%     t1 = sqrt(v_max / jerk); % 加加速过程
%     t3 = 2*t1; % 减加速过程
% else
%     % 这表示极限加速过程中，有一段恒加速度加速过程
%     t1 = a_max/jerk; % 加加速过程
%     t2 = (v_max - a_max^2/jerk)/a_max + t1; % 恒加速过程
%     t3 = t1 + t2; % 减加速过程
% end
% 
% delta_q = 0;
% for t = 0.0001:0.0001:t3
%     n = round(10000*t);
%     if v_max < a_max ^2 / jerk
%         if t <= t1
%             v_ = 0.5 * jerk * t^2;
%         elseif t> t1 && t <= t3
%             v_ = -0.5*jerk * t^2 + jerk * t3 * t+(v_max - 0.5* jerk * t3^2); 
%         else
%             v_ = v_max;
%         end
%     else
%         if t<= t1
%             v_ = 0.5 * jerk * t^2;
%         elseif t <= t2 && t > t1
%             v_ = -0.5 * a_max^2/jerk + a_max * t;
%         elseif t <= t3 && t > t2
%             v_ = -0.5*jerk * t^2 + jerk * t3 * t+(v_max - 0.5* jerk * t3^2);
%         else
%             v_ = v_max;
%         end
%     end      
%     v(n) = v_;
%     delta_q = delta_q + v_*0.0001;
% end
% 
% delta_q
% t = 0.0001:0.0001:t3;
% plot(t,v)