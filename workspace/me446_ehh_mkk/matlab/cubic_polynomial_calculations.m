syms a0 a1 a2 a3

% q0 = 0;
% v0 = 0;
% t0 = 0;
% qf = 0.5;
% vf = 0;
% tf = 1;

q0 = 0.5;
v0 = 0;
t0 = 1;
qf = 0;
vf = 0;
tf = 2;

eqn1 = a0 + a1*t0 + a2*t0^2  + a3*t0^3 == q0;
eqn2 = a1 + 2*a2*t0 + 3*a3*t0^2 == v0;
eqn3 = a0 + a1*tf + a2*tf^2 + a3*tf^3 == qf;
eqn4 = a1 + 2*a2*tf + 3*a3*tf^2 == vf;
[A,B] = equationsToMatrix([eqn1, eqn2, eqn3, eqn4], [a0, a1, a2, a3]);
X = linsolve(A,B)