
%System Dynamic Matrix A (Open Loop System)
%This is a second order system
n = 2; 
A = [0 1
     0 0];
 
%System Control Matrix
B = [0 1]';

%System Output Matrix
C = [2 0];

%Initial State Values
x0 = [1; -2];

%Check For Necessary Conditions
controlability = [B A*B];
observability = [C 
                 C*A]';
G = C'*C;

%Cost Values
R = 2; %Control Cost
Q = [8 0 %State Cost
     0 0] ;
N = 0; 

%Solving For Optimal Feedback Gain
%Making sure both observability and controlability matrices are full rank
if rank(controlability) == n && rank(observability) == n
    [F, K, e] = lqr(A, B, Q, R, N);
end

%Optimal Control
syms x1(t) x2(t) p1(t) p2(t) u(t) t
x = [x1; x2];
u_opt = -F*x;

%Optimal trajectory using closed loop system
time = linspace(0, 10);
x_prime_opt = [x2; u];
x_prime_opt = subs(x_prime_opt, u, u_opt);
x_prime_opt = diff(x) == x_prime_opt;
cond = x(0) == x0;

[x1_opt(t), x2_opt(t)] = dsolve(x_prime_opt, cond);
u_opt = @(t) -F*[x1_opt(t); x2_opt(t)];

%Optimal Performance Criteria J
J = double(int((4*x1_opt.^2) + (2*exp(-t)*cos(t)+2*exp(-t)*sin(t))^2, 0, inf))

%Plot
plot(time, x1_opt(time), time, x2_opt(time), time, u_opt(time))
title('LQR Problem')
legend('x1(t)', 'x2(t)', 'u(t)')
xlabel('Time (seconds)')






