function [F, K, e] = lqr(A, B, Q, R, N)
% u = -Fx
% F = R^-1 * B' * K
%-K*A - A'*K - Q - K*B*(R^-1)*B'*K = 0 is the algebraic riccotti equation
%for infinite time horzion where K prime = 0
syms K
K = sym('K%d%d', [size(A)]); %#ok<NBRAK>
ARE = -K*A - A'*K - Q + K*B*(R^-1)*B'*K == 0;
K = solve(ARE, 'Real', true);
K = struct2cell(K);
K = [K{1:size(K)}];

%Pick a solution that offers stability
for i = 1:size(K) 
    K_opt = double(reshape(K(i,1:4), size(A)));
    F = (R^-1) * (B' * K_opt + N');
    e = eig(A - B*F);
    if all(any(e<0))
        break
    end
end


F = F; %Optimal sate-feedback gain
K = K_opt; %Solution to the riccotti equation
e = e; %Closed loop eig values of system (A-B*K)
end

