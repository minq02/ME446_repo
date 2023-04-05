#include "math.h"
#include "F28335Serial.h"

#define PI          3.1415926535897932384626433832795
#define TWOPI       6.283185307179586476925286766559
#define HALFPI      1.5707963267948966192313216916398
#define GRAV        9.81

typedef struct steptraj_s {
    long double b[5];
    long double a[5];
    long double xk[5];
    long double yk[5];
    float qd_old;
    float qddot_old;
    int size;
} steptraj_t;

steptraj_t trajectory1 = {1.4781526816424225e-07L,5.9126107265696901e-07L,8.8689160898545357e-07L,5.9126107265696901e-07L,1.4781526816424225e-07L,
                        1.0000000000000000e+00L,-3.8431372549019605e+00L,5.5386389850057673e+00L,-3.5476249707880076e+00L,8.5212560572849194e-01L,
                        0,0,0,0,0,
                        0,0,0,0,0,
                        0,
                        0,
                        5};

steptraj_t trajectory2 = {1.4781526816424225e-07L,5.9126107265696901e-07L,8.8689160898545357e-07L,5.9126107265696901e-07L,1.4781526816424225e-07L,
                        1.0000000000000000e+00L,-3.8431372549019605e+00L,5.5386389850057673e+00L,-3.5476249707880076e+00L,8.5212560572849194e-01L,
                        0,0,0,0,0,
                        0,0,0,0,0,
                        0,
                        0,
                        5};

// this function must be called every 1ms.
void implement_discrete_tf(steptraj_t *traj, float step, float *qd, float *qd_dot, float *qd_ddot) {
    int i = 0;

    traj->xk[0] = step;
    traj->yk[0] = traj->b[0]*traj->xk[0];
    for (i = 1;i<traj->size;i++) {
        traj->yk[0] = traj->yk[0] + traj->b[i]*traj->xk[i] - traj->a[i]*traj->yk[i];
    }

    for (i = (traj->size-1);i>0;i--) {
        traj->xk[i] = traj->xk[i-1];
        traj->yk[i] = traj->yk[i-1];
    }

    *qd = traj->yk[0];
    *qd_dot = (*qd - traj->qd_old)*1000;  //0.001 sample period
    *qd_ddot = (*qd_dot - traj->qddot_old)*1000;

    traj->qd_old = *qd;
    traj->qddot_old = *qd_dot;
}

// to call this function create a variable that steps to the new positions you want to go to, pass this var to step
// pass a reference to your qd variable your qd_dot variable and your qd_double_dot variable
// for example
//  implement_discrete_tf(&trajectory, mystep, &qd, &dot, &ddot);



// These two offsets are only used in the main file user_CRSRobot.c  You just need to create them here and find the correct offset and then these offset will adjust the encoder readings
float offset_Enc2_rad = -0.42865286;
float offset_Enc3_rad = 0.22148228;

// Your global varialbes.

long mycount = 0;

#pragma DATA_SECTION(whattoprint, ".my_vars")
float whattoprint = 1820.0;

#pragma DATA_SECTION(myvar, ".my_vars")
float myvar = 1820.0;

#pragma DATA_SECTION(theta1array, ".my_arrs")
float theta1array[100];

#pragma DATA_SECTION(theta2array, ".my_arrs")
float theta2array[100];


long arrayindex = 0;
int UARTprint = 0;

float printtheta1motor = 0;
float printtheta2motor = 0;
float printtheta3motor = 0;

float x_loc = 0;
float y_loc = 0;
float z_loc = 0;

float L = 0.254;

float theta1 = 0;
float theta2 = 0;
float theta3 = 0;

float alpha = 0;
float beta = 0;
float gamma = 0;

float mt1 = 0;
float mt2 = 0;
float mt3 = 0;

// Assign these float to the values you would like to plot in Simulink
float Simulink_PlotVar1 = 0;
float Simulink_PlotVar2 = 0;
float Simulink_PlotVar3 = 0;
float Simulink_PlotVar4 = 0;


//PD variables for theta1
float theta1_old = 0;
float omega1_old1 = 0;
float omega1_old2 = 0;
float omega1 = 0;
float tau1_temp = 0;
float theta1_des = 0;

//PD variables for theta2
float theta2_old = 0;
float omega2_old1 = 0;
float omega2_old2 = 0;
float omega2 = 0;
float tau2_temp = 0;
float theta2_des = 0;

//PD variables for theta3
float theta3_old = 0;
float omega3_old1 = 0;
float omega3_old2 = 0;
float omega3 = 0;
float tau3_temp = 0;
float theta3_des = 0;

//PD control variables
float Kp1 = 24;
float Kp2 = 100;
float Kp3 = 220;
float KD1 = 2.2;
float KD2 = 8;
float KD3 = 10;

float Kp1s = 20;
float Kp2s = 50;
float Kp3s = 70;
float KD1s = 1.5;
float KD2s = 2;
float KD3s = 1.6;

float ethresh1 = 0.02;
float ethresh2 = 0.02;
float ethresh3 = 0.02;

//PID variables
float KI1 = 200;
float KI2 = 400;
float KI3 = 400;

float Ik1 = 0;
float Ik2 = 0;
float Ik3 = 0;

float Ik1_old = 0;
float Ik2_old = 0;
float Ik3_old = 0;

float error1 = 0;
float error2 = 0;
float error3 = 0;

float error1_old = 0;
float error2_old = 0;
float error3_old = 0;

float time = 0;

//Cubic Trajectory
float a1_des = 0;
float a2_des = 0;
float a3_des = 0;

float omega1_des = 0;
float omega2_des = 0;
float omega3_des = 0;

//fun trajectory
float radius = 0;

//Friction Compensation
float u_fric1 = 0;
float vis_pos_1 = 0.18;
float cou_pos_1 = 0.3637;
float vis_neg_1 = 0.195;
float cou_neg_1 = -0.2948;
float slope1 = 3.6;

float u_fric2 = 0;
float vis_pos_2 = 0.26;
float cou_pos_2 = 0.4759;
float vis_neg_2 = 0.287;
float cou_neg_2 = -0.5031;
float slope2 = 3.6;

float u_fric3 = 0;
float vis_pos_3 = 0.2;
float cou_pos_3 = 0.45;
float vis_neg_3 = 0.2132;
float cou_neg_3 = -0.5190;
float slope3 = 3.6;

//inverse Dynamics Control Law
float alpha2 = 0;
float alpha3 = 0;
float p1 = 0.03;
float p2 = 0.0128;
float p3 = 0.0076;
float p4 = 0.0753;
float p5 = 0.0298;

//LAB 3
float mystep12 = 0.;
float mystep3 = -0.;
float qd3 = 0;
float dot3 = 0;
float ddot3 = 0;
float qd12 = 0;
float dot12 = 0;
float ddot12 = 0;
int change = 0; // 0: Feedforward; 1: Inverse Dynamics

float Kp1_id = 100;
float Kp2_id = 2000;
float Kp3_id = 4000;
float KD1_id = 3;
float KD2_id = 100;
float KD3_id = 150;

// This function is called every 1 ms
void lab(float theta1motor,float theta2motor, float theta3motor, float *tau1, float *tau2, float *tau3, int error) {

    // save past states
    if ((mycount%50)==0) {
        theta1array[arrayindex] = theta1motor;
        if (arrayindex >= 100) {
            arrayindex = 0;
        } else {
            arrayindex++;
        }
    }

    if ((mycount%500)==0) {
        UARTprint = 1;
        GpioDataRegs.GPBTOGGLE.bit.GPIO34 = 1; // Blink LED on Control Card
        GpioDataRegs.GPBTOGGLE.bit.GPIO60 = 1; // Blink LED on Emergency Stop Box
    }


// //PD and PID Control
//if ((mycount%2000)==0) {
//    if (theta1_des > 0.1) {
//        theta1_des = 0;
//        theta2_des = 0;
//        theta3_des = 0;
//    } else {
//        theta1_des = PI / 6;
//        theta2_des = PI / 6;
//        theta3_des = PI / 6;
//    }
//}


//    //PID with feedback cubic trajectory
//    time = (mycount%2000) * 0.001;
//
//    if (time > 1) {
//        theta1_des = -2 + 6 * time - 9.0/2 * time*time + time*time*time;
//        theta2_des = -2 + 6 * time - 9.0/2 * time*time + time*time*time;
//        theta3_des = -2 + 6 * time - 9.0/2 * time*time + time*time*time;
//        omega1_des = 6 - 9 * time + 3*time*time;
//        omega2_des = 6 - 9 * time + 3*time*time;
//        omega3_des = 6 - 9 * time + 3*time*time;
//        a1_des = -9 + 6*time;
//        a2_des = -9 + 6*time;
//        a3_des = -9 + 6*time;
//    } else {
//        theta1_des = 3.0/2 * time*time - time*time*time;
//        theta2_des = 3.0/2 * time*time - time*time*time;
//        theta3_des = 3.0/2 * time*time - time*time*time;
//        omega1_des = 3 * time - 3*time*time;
//        omega2_des = 3 * time - 3*time*time;
//        omega3_des = 3 * time - 3*time*time;
//        a1_des = 3 - 6*time;
//        a2_des = 3 - 6*time;
//        a3_des = 3 - 6*time;
//    }

    // heart trajectory

//    time = (mycount%4000) * 0.001;
//    radius = (1 - sin(PI/2*time))*0.1;
//    x_loc = 0.3;
//    y_loc = radius * cos(PI/2*time);
//    z_loc = radius * sin(PI/2*time) + 0.4;
//
//    alpha = atan2(z_loc-L, pow((pow(x_loc,2) + pow(y_loc,2)), 0.5));
//    beta = acos((pow(z_loc-L, 2) + pow(x_loc, 2) + pow(y_loc, 2) - 2*pow(L, 2))/(-2*pow(L,2)));
//    gamma = (PI - beta) / 2.0;
//
//    theta1 = atan2(y_loc,x_loc);
//    theta2 = - (gamma + alpha);
//    theta3 = PI - beta;
//
//    theta1_des = theta1;
//    theta2_des = theta2 + PI/2;
//    theta3_des = theta3 + theta2_des - PI/2;


    //LAB3 fast trajectory
//    if ((mycount%4000)==0) {
//        if (mystep12 > 0.5) {
//            mystep12 = 0.25;
//        } else {
//            mystep12 = 0.85;
//        }
//    }

    if(mycount%8000<4000)
    {
        mystep12 = 0.85;
    }else{
        mystep12 = 0.25;
    }

//    if ((mycount%4000)==0) {
//        if (mystep3 < 0) {
//            mystep3 = 0.3;
//        } else {
//            mystep3 = -0.3;
//        }
//    }

    if(mycount%8000<4000)
    {
        mystep3 = 0.3;
    }else
    {
        mystep3 = -0.3;
    }

    implement_discrete_tf(&trajectory1, mystep12, &qd12, &dot12, &ddot12);
    implement_discrete_tf(&trajectory2, mystep3, &qd3, &dot3, &ddot3);
    theta1_des = qd12;
    theta2_des = qd12;
    theta3_des = qd3;
    omega1_des = dot12;
    omega2_des = dot12;
    omega3_des = dot3;
    a1_des = ddot12;
    a2_des = ddot12;
    a3_des = ddot3;

    //omega1
    omega1 = (theta1motor - theta1_old)/0.001;
    omega1 = (omega1 + omega1_old1 + omega1_old2)/3.0;
    theta1_old = theta1motor;
    omega1_old2 = omega1_old1;
    omega1_old1 = omega1;

    //omega2
    omega2 = (theta2motor - theta2_old)/0.001;
    omega2 = (omega2 + omega2_old2 + omega2_old2)/3.0;
    theta2_old = theta2motor;
    omega2_old2 = omega2_old1;
    omega2_old1 = omega2;

    //omega3
    omega3 = (theta3motor - theta3_old)/0.001;
    omega3 = (omega3 + omega3_old1 + omega3_old2)/3.0;
    theta3_old = theta3motor;
    omega3_old2 = omega3_old1;
    omega3_old1 = omega3;

//    //PD Control short move vs long move
//    if(fabs(theta1_des - theta1motor)<ethresh1)
//    {
//        tau1_temp = Kp1s * (theta1_des - theta1motor) - KD1s * omega1;
//    } else {
//        tau1_temp = Kp1 * (theta1_des - theta1motor) - KD1 * omega1;
//    }
//
//    if(fabs(theta2_des - theta2motor)<ethresh2)
//    {
//        tau2_temp = Kp2s * (theta2_des - theta2motor) - KD2s * omega2;
//    } else {
//        tau2_temp = Kp2 * (theta2_des - theta2motor) - KD2 * omega2;
//    }
//
//
//    if(fabs(theta3_des - theta3motor)<ethresh3)
//    {
//        tau3_temp = Kp3s * (theta3_des - theta3motor) - KD3s * omega3;
//    } else {
//        tau3_temp = Kp3 * (theta3_des - theta3motor) - KD3 * omega3;
//    }

//  //PID Control
//    error1 = (theta1_des - theta1motor);
//    error2 = (theta2_des - theta2motor);
//    error3 = (theta3_des - theta3motor);
//
//    Ik1 = Ik1_old + ((error1+error1_old)/2)*0.001;
//    Ik2 = Ik2_old + ((error2+error2_old)/2)*0.001;
//    Ik3 = Ik3_old + ((error3+error3_old)/2)*0.001;
//
//    error1_old = error1;
//    error2_old = error2;
//    error3_old = error3;
//
//    if(fabs(theta1_des - theta1motor)<ethresh1)
//    {
//        tau1_temp = Kp1s * (theta1_des - theta1motor) - KD1s * omega1 + KI1 * Ik1;
//    } else {
//        tau1_temp = Kp1 * (theta1_des - theta1motor) - KD1 * omega1;// + KI1 * Ik1;
//        Ik1 = 0;
//        Ik1_old = 0;
//    }
//
//    if(fabs(theta2_des - theta2motor)<ethresh2)
//    {
//        tau2_temp = Kp2s * (theta2_des - theta2motor) - KD2s * omega2 + KI2 * Ik2;
//    } else {
//        tau2_temp = Kp2 * (theta2_des - theta2motor) - KD2 * omega2;// + KI2 * Ik2;
//        Ik2 = 0;
//        Ik2_old = 0;
//    }
//
//    if(fabs(theta3_des - theta3motor)<ethresh3)
//    {
//        tau3_temp = Kp3s * (theta3_des - theta3motor) - KD3s * omega3 + KI3 * Ik3;
//    } else {
//        tau3_temp = Kp3 * (theta3_des - theta3motor) - KD3 * omega3;// + KI3 * Ik3;
//        Ik3 = 0;
//        Ik3_old = 0;
//    }


     //ONLY for cubic trajectory

     if (change == 1) { //inverse dynamics
         tau1_temp = 0.0167 * a1_des + Kp1_id * (theta1_des - theta1motor) + KD1_id * (omega1_des - omega1);
         alpha2 = a2_des + Kp2_id * (theta2_des - theta2motor) + KD2_id * (omega2_des - omega2);
         alpha3 = a3_des + Kp3_id * (theta3_des - theta3motor) + KD3_id * (omega3_des - omega3);
         tau2_temp = p1*alpha2-p3*sin(theta3motor-theta2motor)*alpha3 -p3*cos(theta3motor-theta2motor)*omega3*omega3 -p4*GRAV*sin(theta2motor);
         tau3_temp = -p3*sin(theta3motor-theta2motor)*alpha2+p2*alpha3 +p3*cos(theta3motor-theta2motor)*omega2*omega2 -p5*GRAV*cos(theta3motor);
     } else { //feedforward PD
         tau1_temp = 0.0167 * a1_des + Kp1 * (theta1_des - theta1motor) + KD1 * (omega1_des - omega1);
         tau2_temp = 0.03 * a2_des + Kp2 * (theta2_des - theta2motor) + KD2 * (omega2_des - omega2);
         tau3_temp = 0.0128 * a3_des + Kp3 * (theta3_des - theta3motor) + KD3 * (omega3_des - omega3);
     }



    //Implement Friction Compensation
    //theta1dot
    if (omega1 > 0.1) {
        u_fric1 = vis_pos_1 * omega1 + cou_pos_1;
    } else if (omega1 < -0.1) {
        u_fric1 = vis_neg_1 * omega1 + cou_neg_1;
    } else {
        u_fric1 = slope1 * omega1;
    }
    tau1_temp += u_fric1 * 0.8;

    //theta2dot
    if (omega2 > 0.05) {
        u_fric2 = vis_pos_2 * omega2 + cou_pos_2;
    } else if (omega2 < -0.05) {
        u_fric2 = vis_neg_2 * omega2 + cou_neg_2;
    } else {
        u_fric2 = slope2 * omega2;
    }
    tau2_temp += u_fric2 * 0.8;

    //theta3dot
    if (omega3 > 0.05) {
        u_fric3 = vis_pos_3 * omega3 + cou_pos_3;
    } else if (omega3 < -0.05) {
        u_fric3 = vis_neg_3 * omega3 + cou_neg_3;
    } else {
        u_fric3 = slope3 * omega3;
    }
    tau3_temp += u_fric3 * 0.8;

    //Motor torque limitation(Max: 5 Min: -5)
    if (tau1_temp > 4.9) {
        tau1_temp = 4.9;
        Ik1 = Ik1_old;
    } else if (tau1_temp < -4.9) {
        tau1_temp = -4.9;
        Ik1 = Ik1_old;
    }

    if (tau2_temp > 4.9) {
        tau2_temp = 4.9;
        Ik2 = Ik2_old;
    } else if (tau2_temp < -4.9) {
        tau2_temp = -4.9;
        Ik2 = Ik2_old;
    }

    if (tau3_temp > 4.9) {
        tau3_temp = 4.9;
        Ik3 = Ik3_old;
    } else if (tau3_temp < -4.9) {
        tau3_temp = -4.9;
        Ik3 = Ik3_old;
    }

    Ik1_old = Ik1;
    Ik2_old = Ik2;
    Ik3_old = Ik3;

    *tau1 = tau1_temp;
    *tau2 = tau2_temp;
    *tau3 = tau3_temp;

    printtheta1motor = theta1motor;
    printtheta2motor = theta2motor;
    printtheta3motor = theta3motor;

    x_loc = (127.0*cos(theta1motor)*(cos(theta3motor) + sin(theta2motor)))/500.0;
    y_loc = (127.0*sin(theta1motor)*(cos(theta3motor) + sin(theta2motor)))/500.0;
    z_loc = (127.0*cos(theta2motor))/500.0 - (127.0*sin(theta3motor))/500.0 + 127.0/500.0;

    alpha = atan2(z_loc-L, pow((pow(x_loc,2) + pow(y_loc,2)), 0.5));
    beta = acos((pow(z_loc-L, 2) + pow(x_loc, 2) + pow(y_loc, 2) - 2*pow(L, 2))/(-2*pow(L,2)));
    gamma = (PI - beta) / 2.0;

    theta1 = atan2(y_loc,x_loc);
    theta2 = - (gamma + alpha);
    theta3 = PI - beta;

    mt1 = theta1;
    mt2 = theta2 + PI/2;
    mt3 = theta3 + mt2 - PI/2;

    mt1 = mt1*180/PI;
    mt2 = mt2*180/PI;
    mt3 = mt3*180/PI;

    Simulink_PlotVar1 = theta1motor - theta1_des;
    Simulink_PlotVar2 = theta2motor - theta2_des;
    Simulink_PlotVar3 = theta3motor - theta3_des;
    Simulink_PlotVar4 = theta1_des;

    mycount++;
}

void printing(void){
    serial_printf(&SerialA, "%.2f %.2f,%.2f | %.2f,%.2f,%.2f | %.2f,%.2f,%.2f | %.2f,%.2f,%.2f'   \n\r",printtheta1motor*180/PI,printtheta2motor*180/PI,printtheta3motor*180/PI,x_loc,y_loc,z_loc, theta1, theta2, theta3, mt1, mt2, mt3);
    //serial_printf(&SerialA, "%.2f %.2f,%.2f   \n\r",x_loc, y_loc, z_loc);
}
