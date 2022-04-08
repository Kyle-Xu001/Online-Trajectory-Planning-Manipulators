function state = OnlinePlanning(state0,state1)

q0 = state0.q;
q1 = state1.q;
delta = sign(q1 -q0);

q0 = delta * state0.q;
q1 = delta * state1.q;
v0 = delta * state0.dq;
v1 = delta * state1.dq;
a0 = delta * state0.ddq;
a1 = delta * state1.ddq;
j0 = delta * state0.dddq;
j1 = delta * state1.dddq;
T = 0.001;


v_max_ = 5;
a_max_ = 20;
j_max_ = 100;
v_min_ = -5;
a_min_ = -20;
j_min_ = -100;

v_max = (delta+1)/2*v_max_ + (delta-1)/2*v_min_;
a_max = (delta+1)/2*a_max_ + (delta-1)/2*a_min_;
j_max = (delta+1)/2*j_max_ + (delta-1)/2*j_min_;
v_min = (delta+1)/2*v_min_ + (delta-1)/2*v_max_;
a_min = (delta+1)/2*a_min_ + (delta-1)/2*a_max_;
j_min = (delta+1)/2*j_min_ + (delta-1)/2*j_max_;
Ts = 0.0001;

k=0;
size0 = T/Ts;
state.q = zeros(size0,1);
state.dq = zeros(size0,1);
state.ddq = zeros(size0,1);
state.dddq = zeros(size0,1);

state.q(1) = q0;
state.dq(1) = v0;
state.ddq(1) = a0;
state.dddq(1) = j0;

EPSILON=0;

for i = 2:size0
    T_j2a = (a_min - state.ddq(i-1))/j_min;
    T_j2b = (a1 - a_min)/j_max;
    T_d = (v1-state.dq(i-1))/a_min + T_j2a*(a_min - state.ddq(i-1))/(2*a_min) + T_j2b*(a_min-a1)/(2*a_min);
    if T_d < T_j2a + T_j2b
        T_j2a = - state.ddq(i-1)/j_min + sqrt((j_max-j_min)*(state.ddq(i-1)^2*j_max - j_min*(a1^2+2*j_max*(state.dq(i-1)-v1))))/(j_min*(j_min - j_max));
        T_j2b = a1/j_max + sqrt((j_max-j_min)*(state.ddq(i-1)^2*j_max - j_min*(a1^2+2*j_max*(state.dq(i-1)-v1))))/(j_max*(j_max - j_min));
        T_d = T_j2a + T_j2b;
    end
    h = 0.5 * state.ddq(i-1) * T_d^2 + (j_min*T_j2a*(3*T_d^2 - 3*T_d*T_j2a+T_j2a^2)+j_max*T_j2b^3)/6 + T_d*state.dq(i-1);
    if h <= (q1-state.q(i-1)) - EPSILON
        if (state.dq(i-1)-state.ddq(i-1)^2/(2*j_min) < v_max - EPSILON) && (state.ddq(i-1) < a_max - EPSILON)
            state.dddq(i) = j_max;
        elseif (state.dq(i-1)-state.ddq(i-1)^2/(2*j_min) >= v_max + EPSILON) && (state.ddq(i-1) > 0 + EPSILON)
            state.dddq(i) = j_min;
        else
            state.dddq(i) = 0;
        end
        k = 0;
    else
        %if k == 0
            k = i;
        %end
        if (i-k)>=0 && (i-k) <= T_j2a/Ts
            state.dddq(i) = j_min;
        elseif (i-k) <= T_d/Ts && (i-k) >= (T_d - T_j2b)/Ts
            state.dddq(i) = j_max;
        else
            state.dddq(i) = 0;
        end
    end
    
    
    if abs(state.q(i-1) - q1) < 1e-5 && abs(state.dq(i-1) - v1) < 1e-3 && abs(state.ddq(i-1) - a1) < 1e-2
        state.dq(i-1) = 0;
        state.ddq(i-1) = 0;
        state.dddq(i) = 0;
        state.dddq(i-1) = 0;
    end
    state.ddq(i) = state.ddq(i-1) + Ts/2 * (state.dddq(i-1)+state.dddq(i));
    state.dq(i)  = state.dq(i-1)  + Ts/2 * (state.ddq(i-1)+state.ddq(i));
    state.q(i)   = state.q(i-1)   + Ts/2 * (state.dq(i-1)+state.dq(i));
end

state.q    = delta * state.q;
state.dq   = delta * state.dq;
state.ddq  = delta * state.ddq;
state.dddq = delta * state.dddq;

