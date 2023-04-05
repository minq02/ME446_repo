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

steptraj_t trajectory = {1.4781526816424225e-07L,5.9126107265696901e-07L,8.8689160898545357e-07L,5.9126107265696901e-07L,1.4781526816424225e-07L,
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

//PD Gain variables
float Kpx = 400;
float Kpy = 600;
float Kpz = 600;
float KDx = 5;
float KDy = 5;
float KDz = 5;

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

//Rotation XYZ and Transpose Variables
float cosq1 = 0;
float sinq1 = 0;
float cosq2 = 0;
float sinq2 = 0;
float cosq3 = 0;
float sinq3 = 0;
float JT_11 = 0;
float JT_12 = 0;
float JT_13 = 0;
float JT_21 = 0;
float JT_22 = 0;
float JT_23 = 0;
float JT_31 = 0;
float JT_32 = 0;
float JT_33 = 0;
float cosz = 0;
float sinz = 0;
float cosx = 0;
float sinx = 0;
float cosy = 0;
float siny = 0;
float thetaz = 0;
float thetax = 0;
float thetay = 0;
float R11 = 0;
float R12 = 0;
float R13 = 0;
float R21 = 0;
float R22 = 0;
float R23 = 0;
float R31 = 0;
float R32 = 0;
float R33 = 0;
float RT11 = 0;
float RT12 = 0;
float RT13 = 0;
float RT21 = 0;
float RT22 = 0;
float RT23 = 0;
float RT31 = 0;
float RT32 = 0;
float RT33 = 0;

float x_motor = 0;
float y_motor = 0;
float z_motor = 0;

float x_des = 0;
float y_des = 0;
float z_des = 0;

float x_dot_des = 0;
float y_dot_des = 0;
float z_dot_des = 0;

float x_dot_motor = 0;
float y_dot_motor = 0;
float z_dot_motor = 0;

float Fx = 0;
float Fy = 0;
float Fz = 0;

//friction multiplication factor
float ff1 = 0.5;
float ff2 = 0.5;
float ff3 = 0.5;

//torque constant
float Kt = 6.0;

//Force in Z direction
float F_ZCmd = 0;


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

    //desired end-effector position declaration
    x_des = 0.254;
    y_des = 0.254;
    z_des = 0.254;

    //omega1 calculations
    omega1 = (theta1motor - theta1_old)/0.001;
    omega1 = (omega1 + omega1_old1 + omega1_old2)/3.0;
    theta1_old = theta1motor;
    omega1_old2 = omega1_old1;
    omega1_old1 = omega1;

    //omega2 calculations
    omega2 = (theta2motor - theta2_old)/0.001;
    omega2 = (omega2 + omega2_old2 + omega2_old2)/3.0;
    theta2_old = theta2motor;
    omega2_old2 = omega2_old1;
    omega2_old1 = omega2;

    //omega3 calculations
    omega3 = (theta3motor - theta3_old)/0.001;
    omega3 = (omega3 + omega3_old1 + omega3_old2)/3.0;
    theta3_old = theta3motor;
    omega3_old2 = omega3_old1;
    omega3_old1 = omega3;

    // Jacobian Transpose
    cosq1 = cos(theta1motor);
    sinq1 = sin(theta1motor);
    cosq2 = cos(theta2motor);
    sinq2 = sin(theta2motor);
    cosq3 = cos(theta3motor);
    sinq3 = sin(theta3motor);
    JT_11 = -0.254*sinq1*(cosq3 + sinq2);
    JT_12 = 0.254*cosq1*(cosq3 + sinq2);
    JT_13 = 0;
    JT_21 = 0.254*cosq1*(cosq2 - sinq3);
    JT_22 = 0.254*sinq1*(cosq2 - sinq3);
    JT_23 = -0.254*(cosq3 + sinq2);
    JT_31 = -0.254*cosq1*sinq3;
    JT_32 = -0.254*sinq1*sinq3;
    JT_33 = -0.254*cosq3;

    //Forward Kinematic Equations
    x_motor = 0.254 * cosq1 * (cosq3 + sinq2);
    y_motor = 0.254 * sinq1 * (cosq3 + sinq2);
    z_motor = 0.254 * (1 + cosq2 - sinq3);

    //end effector velocity calculation using jacobians
    x_dot_motor = JT_11 * omega1 + JT_21 * omega2 + JT_31 * omega3;
    y_dot_motor = JT_12 * omega1 + JT_22 * omega2 + JT_32 * omega3;
    z_dot_motor = JT_13 * omega1 + JT_23 * omega2 + JT_33 * omega3;

    //Task Space PD Control
    Fx = Kpx * (x_des - x_motor) + KDx * (x_dot_des - x_dot_motor);
    Fy = Kpy * (y_des - y_motor) + KDy * (y_dot_des - y_dot_motor);
    Fz = Kpz * (z_des - z_motor) + KDz * (z_dot_des - z_dot_motor);

    tau1_temp = JT_11 * Fx + JT_12 * Fy + JT_13 * Fz;
    tau2_temp = JT_21 * Fx + JT_22 * Fy + JT_23 * Fz;
    tau3_temp = JT_31 * Fx + JT_32 * Fy + JT_33 * Fz;

    //Implement Friction Compensation
    //theta1dot
    if (omega1 > 0.1) {
        u_fric1 = vis_pos_1 * omega1 + cou_pos_1;
    } else if (omega1 < -0.1) {
        u_fric1 = vis_neg_1 * omega1 + cou_neg_1;
    } else {
        u_fric1 = slope1 * omega1;
    }
    tau1_temp += (u_fric1 * ff1 + JT_31 * (F_ZCmd / Kt));

    //theta2dot
    if (omega2 > 0.05) {
        u_fric2 = vis_pos_2 * omega2 + cou_pos_2;
    } else if (omega2 < -0.05) {
        u_fric2 = vis_neg_2 * omega2 + cou_neg_2;
    } else {
        u_fric2 = slope2 * omega2;
    }
    tau2_temp += (u_fric2 * ff2 + JT_32 * (F_ZCmd / Kt));

    //theta3dot
    if (omega3 > 0.05) {
        u_fric3 = vis_pos_3 * omega3 + cou_pos_3;
    } else if (omega3 < -0.05) {
        u_fric3 = vis_neg_3 * omega3 + cou_neg_3;
    } else {
        u_fric3 = slope3 * omega3 + JT_33 * (F_ZCmd / Kt);
    }
    tau3_temp += (u_fric3 * ff3);

    //Motor torque saturations(Max: 4.9; Min: -4.9)
    if (tau1_temp > 4.9) {
        tau1_temp = 4.9;
    } else if (tau1_temp < -4.9) {
        tau1_temp = -4.9;
    }

    if (tau2_temp > 4.9) {
        tau2_temp = 4.9;
    } else if (tau2_temp < -4.9) {
        tau2_temp = -4.9;
    }

    if (tau3_temp > 4.9) {
        tau3_temp = 4.9;
    } else if (tau3_temp < -4.9) {
        tau3_temp = -4.9;
    }

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

    Simulink_PlotVar1 = theta1motor;
    Simulink_PlotVar2 = theta2motor;
    Simulink_PlotVar3 = theta3motor;
    Simulink_PlotVar4 = theta1_des;

    mycount++;
}

void printing(void){
    serial_printf(&SerialA, "%.2f %.2f,%.2f | %.2f,%.2f,%.2f | %.2f,%.2f,%.2f | %.2f,%.2f,%.2f'   \n\r",printtheta1motor*180/PI,printtheta2motor*180/PI,printtheta3motor*180/PI,x_loc,y_loc,z_loc, theta1, theta2, theta3, mt1, mt2, mt3);
    //serial_printf(&SerialA, "%.2f %.2f,%.2f   \n\r",x_loc, y_loc, z_loc);
}
