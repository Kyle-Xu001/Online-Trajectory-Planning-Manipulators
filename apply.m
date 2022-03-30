function q_cmd = apply(dq_des)
% 常量
njoints_ = evalin('base','njoints_');
q_min_   = evalin('base','q_min_');
q_max_   = evalin('base','q_max_');
dq_max_  = evalin('base','dq_max_');
ddq_max_ = evalin('base','ddq_max_');
delta_t_ = evalin('base','delta_t_');
delta_q_min_ = evalin('base','delta_q_min_');
delta_q_acc_ = evalin('base','delta_q_acc_');

q_final_ = evalin('base','q_final_');
q_cmd_ = evalin('base','q_cmd_');
q_cmd_prev_ = evalin('base','q_cmd_prev_');

dq_des_ = evalin('base','dq_des_');
dq_des_prev_ = evalin('base','dq_des_prev_');
delta_q_ = evalin('base','delta_q_');
delta_q_max_ = evalin('base','delta_q_max_');
sign_ = evalin('base','sign_');
dist_to_final_ = evalin('base','dist_to_final_');
dist_ad_ = evalin('base','dist_ad_');
flag_speed_ = evalin('base','flag_speed_');
status_ = evalin('base','status_');


for i = 1:njoints_
    dq_des_(i) = dq_des(i);
    if dq_des_(i) ~= dq_des_prev_(i)
        flag_joint_limit_ = 0;
        if dq_des_(i) >dq_max_(i)
            dq_des_(i) = dq_max_(i);
        elseif -dq_des_(i) > dq_max_(i)
            dq_des_(i) = -dq_max_(i);
        end

        if flag_speed_(i) == 0
            if status_(i) == 0
                if dq_des_(i) > 0
                    delta_q_max_(i) = dq_des_(i) * delta_t_;
                    sign_(i) = 1;
                    q_final_(i) = q_max_(i);
                    delta_q_(i) = 0;
                    status_(i) = 1;
                elseif dq_des_(i) < 0
                    delta_q_max_(i) = -dq_des_(i) * delta_t_;
                    sign_(i) = -1;
                    q_final_(i) = q_min_(i);
                    delta_q_(i) = 0;
                    status_(i) = 1;
                end
            elseif dq_des_(i)*sign_(i) <0
                flag_speed_(i)=1;
                status_(i) = 2;
                delta_q_max_(i) = 0;
            else
                if sign_(i) ==1
                    if dq_des_(i) > dq_des_prev_(i)
                        status_(i) = 1;
                    else
                        status_(i) = 2;
                    end
                    delta_q_max_(i) = dq_des_(i) * delta_t_;
                else
                    if dq_des_(i)>dq_des_prev_(i)
                        status_(i) = 2;
                    else
                        status_(i) = 1;
                    end
                    delta_q_max_(i) = -dq_des(i)*delta_t_;
                end
            end
            n = fix(delta_q_max_(i)/delta_q_acc_(i));
            dist_ad_(i) = n*(delta_q_max_(i) - (n+1)*delta_q_acc_(i)/2);
        end
        dq_des_prev_(i) = dq_des_(i);
    end
end

for i = 1:njoints_
    dist_to_final_(i) = (q_final_(i) - q_cmd_(i)) * sign_(i);
    if (dist_to_final_(i) - delta_q_max_(i)) <= dist_ad_(i)
        if dist_ad_(i) > 0
            flag_joint_limit_ = 1;
            for k = 1:njoints_
                if status_(k) ~= 0
                    status_(k) = 2;
                end
                delta_q_max_(k) = 0;
            end
        end
    end

    if status_(i) == 2
        delta_q_(i) = delta_q_(i) - delta_q_acc_(i);
        if delta_q_(i) <= delta_q_max_(i)
            if delta_q_max_(i) < delta_q_min_
                status_(i) = 0;
                delta_q_(i) = 0;
                if flag_speed_(i) == 1
                    if dq_des_(i) >0
                        delta_q_max_(i) = dq_des_(i) * delta_t_;
                        sign_(i) = 1;
                        q_final_(i) = q_max_(i);    
                    elseif dq_des_(i) < 0
                        delta_q_max_(i) = -dq_des_(i) * delta_t_;
                        sign_(i) = -1;
                        q_final_(i) = q_min_(i);
                    end
                    status_(i) = 1;
                    flag_speed_(i) = 0;
                    n = fix(delta_q_max_(i)/delta_q_acc_(i));
                    dist_ad_(i) = n * (delta_q_max_(i) - (n+1)*delta_q_acc_(i)/2);
                end
            elseif delta_q_max_(i)>0 && flag_joint_limit_==0
                if delta_q_max_(i) < (delta_q_(i) + 2*delta_q_acc_(i))
                    delta_q_(i) = delta_q_max_(i);
                    status_(i) = 3;
                elseif flag_joint_limit_ == 0
                    delta_q_(i) = delta_q_(i)+ 2*delta_q_acc_(i);
                    status_(i) = 1;
                end
            end
        end
    elseif status_(i) == 1
        delta_q_(i) = delta_q_(i) + delta_q_acc_(i);
        if delta_q_(i) >= delta_q_max_(i)
            delta_q_(i) = delta_q_max_(i);
            status_(i) = 3;
        end
    end
    q_cmd_(i) = q_cmd_(i) + sign_(i) * delta_q_(i);
end

for i = 1:njoints_
    butee = q_min_(i);
    if q_cmd_(i) < butee
        for j = 1:njoints_
            q_cmd_(j) = q_cmd_(j)- sign_(j)*delta_q_(j);
        end
        q_cmd_(i) = butee;
        break;
    end
    butee = q_max_(i);
    if q_cmd_(i) > butee
        for j = 1:njoints_
            q_cmd_(j) = q_cmd_(j)- sign_(j)*delta_q_(j);
        end
        q_cmd_(i) = butee;
        break;
    end
end

q_cmd = q_cmd_;
q_cmd_prev_ = q_cmd_;

assignin('base','q_final_',q_final_);
assignin('base','q_cmd_',q_cmd_);
assignin('base','q_cmd_prev_',q_cmd_prev_);
assignin('base','dq_des_',dq_des_);
assignin('base','dq_des_prev_',dq_des_prev_);
assignin('base','delta_q_',delta_q_);
assignin('base','delta_q_max_',delta_q_max_);
assignin('base','sign_',sign_);
assignin('base','dist_to_final_',dist_to_final_);
assignin('base','dist_ad_',dist_ad_);
assignin('base','flag_speed_',flag_speed_);
assignin('base','status_',status_);




    


        
    

    
            
                  
                 
        
        
