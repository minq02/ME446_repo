clc
clear

syms t2 t3 w2 w3 a2 a3;

theta = [t2; t3];
omega = [w2; w3];
alpha = [a2; a3];

g = 9.81;

p = [0.03; 0.0128; 0.0076; 0.0753; 0.0298];

D = [p(1), -p(3)*sin(t3 - t2);
     -p(3)*sin(t3 - t2), p(2)];

C = [0, -p(3)*cos(t3 - t2)*w3;
     p(3)*cos(t3 - t2)*w2, 0];

G = [-p(4)*g*sin(t2); -p(5)*g*cos(t3)];

t = D*alpha + C*omega + G;