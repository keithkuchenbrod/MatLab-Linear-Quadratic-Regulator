
%This is a second order system
n = 2; 

%System State Matrix
A =  [1 1
      0 1];
 
%System Control Matrix
B = [0 1]';

%System Output Matrix
C = [2 0];

%System Feedthrough Matrix
D = 0;

%Initial State Values
x0 = [1; -2];

%Check For Necessary Conditions
controlability_m = [B A*B];
observability_m = [C 
                 C*A]';
G = C'*C;

%Cost Values
R = 2; %Control Cost
Q = [8 0 %State Cost
     0 0] ;
N = 0; 

%Open-Loop Eigenvalues
OL_Eig = eig(A);

%Solving For Optimal Feedback Gain
%Making sure both observability and controlability matrices are full rank
if rank(controlability_m) == n && rank(observability_m) == n
    [F, K, e] = lqr(A, B, Q, R, N);
end

%Unity Feedback gain/ Chosen Feedback gain (Comment out to stop)
%This just to interupt the feedback gain F calculated from lqr()
%F = [5 5];

%Riccati solution
riccati_solution_K = K

%Closed loop eigen values
CL_eig = eig(A-B*F)

%Optimal Control
syms x1(t) x2(t) p1(t) p2(t) u(t) t
x = [x1; x2];
system = A*x + B*u
u_opt = -F*x

%Optimal trajectory using closed loop system
time = linspace(0, 10);
closed_loop_system = subs(system, u, u_opt);
closed_loop_system = diff(x) == closed_loop_system;
cond = x(0) == x0;

[x1_opt(t), x2_opt(t)] = dsolve(closed_loop_system, cond);
u_opt = @(t) -F*[x1_opt(t); x2_opt(t)];

%Optimal Performance Criteria J
J = double(int((4*x1_opt.^2) + (2*exp(-t)*cos(t)+2*exp(-t)*sin(t))^2, 0, inf))

%Plot
plot(time, x1_opt(time), time, x2_opt(time), time, u_opt(time))
title('LQR Problem')
legend('x1(t)', 'x2(t)', 'u(t)')
xlabel('Time (seconds)')






