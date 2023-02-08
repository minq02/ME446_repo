x = 0.3;
y = -0.3;
z = 0.254;
L = 0.254;

alpha = atan2(z-L, (x^2 + y^2)^0.5);
beta = acos(((z-L)^2 + x^2 + y^2 - 2*L^2)/(-2*L^2));
gamma = (pi - beta) / 2.0;

theta1 = atan2(y,x);
theta2 = - (gamma + alpha);
theta3 = pi - beta;

mt1 = theta1;
mt2 = theta2 + pi/2;
mt3 = theta3 + mt2 - pi/2;

mt1*180/pi
mt2*180/pi
mt3*180/pi