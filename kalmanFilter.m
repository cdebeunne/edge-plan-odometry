function [X, P] = kalmanFilter(X, P, u, z, Q, R)
% Computation of Kalman Filter

%% prediction step 

Xp1 =  [u+X(7:12);
    X(7:12)];
F = [zeros(6,6), eye(6,6);
    zeros(6,6), eye(6,6)];
Pp1 = F*P*F' + Q;


%% correction step 

h = Xp1(1:6);
H = [eye(6,6), zeros(6,6)];
mu = z - H*Xp1;
S = H*Pp1*H' + R;
K = Pp1*H'*inv(S);
X = Xp1 + K*mu;
P = Pp1 - K*H*Pp1;
end