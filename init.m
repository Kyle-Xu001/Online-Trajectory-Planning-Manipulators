function init(q_cur)
njoints_ = 7;
q_min_   = [-160/180*pi, -110/180*pi, -160/180*pi, -110/180*pi, -160/180*pi, -110/180*pi, -350/180*pi];
q_max_   = [ 160/180*pi,  110/180*pi,  160/180*pi,  110/180*pi,  160/180*pi,  110/180*pi,  350/180*pi];
dq_max_  = [ 4, 4, 5, 5, 6, 6, 7];
ddq_max_ = [ 150, 150, 150, 200, 200, 200, 200];
delta_t_ = 0.001;
q_final_ = q_cur;
q_cmd_ = q_cur;
q_cmd_prev_ = q_cmd_;

dq_des_ = [0 0 0 0 0 0 0];
dq_des_prev_ = [0 0 0 0 0 0 0];
delta_q_ = [0 0 0 0 0 0 0];
delta_q_max_ = [0 0 0 0 0 0 0];
sign_ = [0 0 0 0 0 0 0];
dist_to_final_ = [0 0 0 0 0 0 0];
dist_ad_ = [0 0 0 0 0 0 0];
flag_speed_ = [0 0 0 0 0 0 0];
status_ = [0 0 0 0 0 0 0];
delta_q_acc_ = [0 0 0 0 0 0 0];
delta_q_min_ = 1e-4;

for i = 1:njoints_
    delta_q_acc_(i) = ddq_max_(i) *delta_t_ *delta_t_;
end

assignin('base','njoints_',njoints_);
assignin('base','q_min_',q_min_);
assignin('base','q_max_',q_max_);
assignin('base','dq_max_',dq_max_);
assignin('base','ddq_max_',ddq_max_);
assignin('base','delta_t_',delta_t_);
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
assignin('base','delta_q_min_',delta_q_min_);
assignin('base','delta_q_acc_',delta_q_acc_);

