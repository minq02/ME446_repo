syms motor_theta1 motor_theta2 motor_theta3;
syms t1 t2 t3;
a1 = 0;
a2 = 0.254;
a3 = 0.254;
alpha1 = sym(-pi/2);
alpha2 = 0;
alpha3 = 0;
d1 = 0.254;
d2 = 0;
d3 = 0;
t1 = motor_theta1;
t2 = 1*motor_theta2 - sym(pi/2);
t3 = -1*motor_theta2 + 1*motor_theta3 + sym(pi/2);

a1 = [cos(t1), -sin(t1)*cos(alpha1), sin(t1)*sin(alpha1), a1*cos(t1);
    sin(t1), cos(t1)*cos(alpha1), -cos(t1)*sin(alpha1), a1*sin(t1);
    0, sin(alpha1), cos(alpha1), d1;
    0, 0, 0, 1];
a2 = [cos(t2), -sin(t2)*cos(alpha2), sin(t2)*sin(alpha2), a2*cos(t2);
    sin(t2), cos(t2)*cos(alpha2), -cos(t2)*sin(alpha2), a2*sin(t2);
    0, sin(alpha2), cos(alpha2), d2;
    0, 0, 0, 1];
a3 = [cos(t3), -sin(t3)*cos(alpha3), sin(t3)*sin(alpha3), a3*cos(t3);
    sin(t3), cos(t3)*cos(alpha3), -cos(t3)*sin(alpha3), a3*sin(t3);
    0, sin(alpha3), cos(alpha3), d3;
    0, 0, 0, 1];

T0_3 = a1*a2*a3;
subs(T0_3, [t1, t2, t3], [0, 0, 0])
simplify(T0_3)

%eqn1 = c1*sym(pi/2) + c2*0 + c3 == 0;
%eqn2 = c1*0 + c2*0 + c3 == sym(pi/2);
%eqn3 = c1*0 - c2*sym(pi/2) + c3 == 0;
%[A,B] = equationsToMatrix([eqn1, eqn2, eqn3], [c1, c2, c3]);
%X = linsolve(A,B);
%c1 = -1; c2 = 1; c3 = pi/2



