function [X, P] = kalmanFilter(X, P, u, z, Q, R)

%%% prediction step %%%

Xp1 = [X(1) + u(1)*cos(X(3)) - u(2)*sin(X(3)) + X(4);
    X(2) + u(2)*cos(X(3)) + u(1)*sin(X(3)) + X(5);
    X(3) + u(3) + X(6);
    X(4); X(5); X(6)];
F = eye(6,6);
F(1,3) = -u(1)*sin(X(3)) - u(2)*cos(X(3));
F(2,3) = u(1)*cos(X(3)) -u(2)*sin(X(3));
F(1:3, 4:6) = eye(3,3);

Pp1 = F*P*F' + Q;

%%% correction step %%%

h = [(Xp1(1)-X(1))*cos(X(3)) + (Xp1(2)-X(2))*sin(X(3));
    -(Xp1(1)-X(1))*sin(X(3)) + (Xp1(2)-X(2))*cos(X(3));
    Xp1(3)-X(3)];
Hp1 = zeros(3, 6);
Hp1(1:3, 1:3) = [cos(X(3)) sin(X(3)) 0;
    -sin(X(3)) +cos(X(3)) 0;
    0 0 1];
mu = z - h;
S = Hp1*Pp1*Hp1'+ R;
K = Pp1*Hp1'*inv(S);
X = Xp1 + K*mu;
P = Pp1 - K*Hp1*Pp1;
end
