clear
clc
%% Coefficient equations
syms t a0 a1 a2 a3 a4 a5
% diff
p = @(t) a5*t^5 + a4*t^4 + a3*t^3 + a2*t^2 + a1*t + a0;
dp = @(t) 5*a5*t^4 + 4*a4*t^3 + 3*a3*t^2 + 2*a2*t + a1;
ddp = @(t) 20*a5*t^3 + 12*a4*t^2 + 6*a3*t + 2*a2;

syms T p_start p_end v_start v_end a_start a_end
eqs = [  p(0) == p_start;   p(T) == p_end;
        dp(0) == v_start;  dp(T) == v_end;
       ddp(0) == a_start; ddp(T) == a_end];
cfs = solve(eqs,[a0, a1, a2, a3, a4, a5])
%% Scaling T to fit within ddf limit
%ddf_max = rand(1)
p_start = 0; p_end = 100;
v_start = 0; v_end = 0;
a_start = 0; a_end = 0;
time_array = 0:1/100:1;
ddf = @(t, T) a_start - (10*t.^3*(12*p_start - 12*p_end + 6*T*v_end + 6*T*v_start - T.^2*a_end + T.^2*a_start))/T.^5 + (6*t.^2*(30*p_start - 30*p_end + 14*T*v_end + 16*T*v_start - 2*T.^2*a_end + 3*T.^2*a_start))/T.^4 - (3*t*(20*p_start - 20*p_end + 8*T*v_end + 12*T*v_start - T.^2*a_end + 3*T.^2*a_start))/T.^3;
[~,~,ddf2] = QuinticPolynomial([p_start, v_start, a_start; p_start, v_start, a_start],[p_end, v_end, a_end; p_end, v_end, a_end], time_array);

acc_range = 0.001:0.001:10;
output_max = zeros([1,length(acc_range)]);
output_iter = zeros([2,length(acc_range)]);
index = 1;
for ddf_max = acc_range
    T = 1;
    if max(ddf(0:T/100:T, T)) > ddf_max
        % Find upper and lower bound for binary search
        iterationES = 1;
        while 1
            T = T * 2;
            if max(abs(ddf(0:T/100:T, T))) < ddf_max
                T_upper = T;
                T_lower = T / 2;
                break
            end
            iterationES = iterationES + 1;
        end
        % Perform binary search
        iterationBS = 1;
        while 1
            T = T_lower + (T_upper - T_lower)/2;
            if max(abs(ddf(0:T/100:T, T))) <= ddf_max && max(abs(ddf(0:T/100:T, T))) >= ddf_max - 0.01
                % Done
                output_max(index) = max(abs(ddf(0:T/100:T, T)));
                output_iter(:,index) = [iterationES; iterationBS];
                index = index + 1;
                break
            end
            if max(abs(ddf(0:T/100:T, T))) > ddf_max
                T_lower = T;
            elseif max(abs(ddf(0:T/100:T, T))) < ddf_max
                T_upper = T;
            end
            iterationBS = iterationBS + 1;
        end
    end
end

fig1 = figure;
clf;
hold on
plot([0,10],[0,10])
plot(acc_range,output_max)
hold off

fig2 = figure;
clf;
hold on
plot(acc_range,output_iter(1, :))
plot(acc_range,output_iter(2, :))
hold off