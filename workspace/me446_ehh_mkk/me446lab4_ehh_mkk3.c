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


//Forward Kinematic Variables For TerraTerm
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

//Simulink Plot Variables
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

//PD Gain Lab 4 Part 3
float Kpxn = 400;
float Kpyn = 600;
float Kpzn = 600;
float KDxn = 5;
float KDyn = 5;
float KDzn = 5;

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

float d1 = 0;
float d2 = 0;
float d3 = 0;
float p1 = 0;
float p2 = 0;
float p3 = 0;

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

float Fxn = 0;
float Fyn = 0;
float Fzn = 0;

//friction multiplication factor
float ff1 = 0.5;
float ff2 = 0.5;
float ff3 = 0.5;

//torque constant
float Kt = 6.0;

//Force in Z direction
float F_ZCmd = 0;

//Equation of a straight line
float xa = 0.0;
float xb = 0.0;
float ya = 0.0;
float yb = 0.0;
float za = 0.0;
float zb = 0.0;

float t_start = 0.0;
float t_total = 0.0;
float t = 0.0;

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

    /*
     * Straight Line Following
     *
     * To make the robot follow a straight line with a desired speed along that line's direction,
     * begin by defining the desired starting and ending end-effector position.
     *
     * Then, define the time required for one straight line following.
     *
     * After that, write a if-else statement depending on which direction of the path to follow.
     *
     * The three equations for x, y, z as a function of time can be derived given the start and end points
     * along with the desired speed in meters/second
     *
     * Desired speed can be calculated by distance between start and end points multiplied by time taken / time total
     */

    //Desired end-effector position declaration (a: starting | b: ending)
    xa = 0.3;
    xb = 0;
    ya = 0;
    yb = 0.3;
    za = 0.254;
    zb = 0.254;

    //Time required
    t_total = 6.0;

    //Time taken reset
    t = mycount%6000 / 1000.0;

    if (mycount%12000 < 6000) { //CW
        x_des = (xb - xa) * (t - t_start) / t_total + xa;
        y_des = (yb - ya) * (t - t_start) / t_total + ya;
        z_des = (zb - za) * (t - t_start) / t_total + za;
    } else { //CCW
        x_des = (xa - xb) * (t - t_start) / t_total + xb;
        y_des = (ya - yb) * (t - t_start) / t_total + yb;
        z_des = (za - zb) * (t - t_start) / t_total + zb;
    }

    /*
     * Calculating omega by getting the time difference between two theta values.
     * Filtered data using the average value method.
     */

    //Omega1 calculations
    omega1 = (theta1motor - theta1_old)/0.001;
    omega1 = (omega1 + omega1_old1 + omega1_old2)/3.0;
    theta1_old = theta1motor;
    omega1_old2 = omega1_old1;
    omega1_old1 = omega1;

    //Omega2 calculations
    omega2 = (theta2motor - theta2_old)/0.001;
    omega2 = (omega2 + omega2_old2 + omega2_old2)/3.0;
    theta2_old = theta2motor;
    omega2_old2 = omega2_old1;
    omega2_old1 = omega2;

    //Omega3 calculations
    omega3 = (theta3motor - theta3_old)/0.001;
    omega3 = (omega3 + omega3_old1 + omega3_old2)/3.0;
    theta3_old = theta3motor;
    omega3_old2 = omega3_old1;
    omega3_old1 = omega3;

    /*
     * Implement Task Space PD Control
     *
     * In order to implement a task space control law with friction compensation,
     * begin by defining the transpose of the Jacobian and forward kinematic equations for the CRS robot
     *
     * Then, calculate the end effector velocity (x_dot_motor, y_dot_motor, z_dot_motor) using Jacobians.
     * By definition, jacobians link between angular velocities of joints and xyz velocities of end effector.
     */

    //Jacobian Transpose Definition provided in lab 4 manual
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

    //Forward Kinematic Equations provided in lab 4 manual
    x_motor = 0.254 * cosq1 * (cosq3 + sinq2);
    y_motor = 0.254 * sinq1 * (cosq3 + sinq2);
    z_motor = 0.254 * (1 + cosq2 - sinq3);

    //End effector velocity calculation using jacobians
    x_dot_motor = JT_11 * omega1 + JT_21 * omega2 + JT_31 * omega3;
    y_dot_motor = JT_12 * omega1 + JT_22 * omega2 + JT_32 * omega3;
    z_dot_motor = JT_13 * omega1 + JT_23 * omega2 + JT_33 * omega3;

    /*
     * Feedforward Force
     *
     * We will apply a force with the robot in the direction of its Z world direction.
     *
     * Begin by initializing and defining Kt, which is the robot's torque constant.
     *
     * Calculate and add the vector multiplication of tranpose of Jacobian with [0; 0; force_z / Kt]
     * to convert from xyz of end effector to join torques.
     */

//    //Task Space PD Control ONLY for feedforward
//    //Force in xyz using PD Control
//    Fx = Kpx * (x_des - x_motor) + KDx * (x_dot_des - x_dot_motor);
//    Fy = Kpy * (y_des - y_motor) + KDy * (y_dot_des - y_dot_motor);
//    Fz = Kpz * (z_des - z_motor) + KDz * (z_dot_des - z_dot_motor);
//
//    //Conversion to tau 1, 2, and 3 using jacobian matrix
//    tau1_temp = JT_11 * Fx + JT_12 * Fy + JT_13 * Fz + JT_31 * (F_ZCmd / Kt);
//    tau2_temp = JT_21 * Fx + JT_22 * Fy + JT_23 * Fz + JT_32 * (F_ZCmd / Kt);
//    tau3_temp = JT_31 * Fx + JT_32 * Fy + JT_33 * Fz + JT_33 * (F_ZCmd / Kt);

    /*
     * Impedance Control in non world frame axis
     *
     * Impedance is the relationship of force and displacement (and its derivatives)
     *
     * Begin by defining a desired frame, N, that you want to apply impedance control on.
     * Frame N is defined by thetax, thetay, and thetaz, the rotation on world frame, W
     * Also define the rotational matrix and its transpose
     *
     * We will use the rotation matrix coordinate transformation to select frame N as the weak axis.
     * We can also use the rotation matrix to perform coordinate transformation of Fx, Fy, Fz in N frame
     * to those of world frame.
     *
     * Since it's easier to think about commanding the arm at world x, y, z coordinate point, we will rotate
     * the errors from World coordinate to the N frame using the rotational matrix again.
     */

    // Desired N Frame for Impedance Control
    thetax = 0;
    thetay = 0;
    thetaz = PI/4;

    // Rotation xyz and its Transpose
    cosz = cos(thetaz);
    sinz = sin(thetaz);
    cosx = cos(thetax);
    sinx = sin(thetax);
    cosy = cos(thetay);
    siny = sin(thetay);
    RT11 = R11 = cosz*cosy-sinz*sinx*siny;
    RT21 = R12 = -sinz*cosx;
    RT31 = R13 = cosz*siny+sinz*sinx*cosy;
    RT12 = R21 = sinz*cosy+cosz*sinx*siny;
    RT22 = R22 = cosz*cosx;
    RT32 = R23 = sinz*siny-cosz*sinx*cosy;
    RT13 = R31 = -cosx*siny;
    RT23 = R32 = sinx;
    RT33 = R33 = cosx*cosy;

    //ONLY for impedance control in a new frame N
    //Define the positional part for PD control and transform coordinates using rotational matrix
    p1 = Kpxn * (R11 * (x_des - x_motor) + R12 * (y_des - y_motor) + R13 * (z_des - z_motor));
    p2 = Kpyn * (R21 * (x_des - x_motor) + R22 * (y_des - y_motor) + R23 * (z_des - z_motor));
    p3 = Kpzn * (R31 * (x_des - x_motor) + R32 * (y_des - y_motor) + R33 * (z_des - z_motor));

    //Define the derivative part for PD control and transform coordinates using rotational matrix
    d1 = KDxn * (R11 * (x_dot_des - x_dot_motor) + R12 * (y_dot_des - y_dot_motor) + R13 * (z_dot_des - z_dot_motor));
    d2 = KDyn * (R21 * (x_dot_des - x_dot_motor) + R22 * (y_dot_des - y_dot_motor) + R23 * (z_dot_des - z_dot_motor));
    d3 = KDzn * (R31 * (x_dot_des - x_dot_motor) + R32 * (y_dot_des - y_dot_motor) + R33 * (z_dot_des - z_dot_motor));

    //Force in xyz using PD Control and transform from N frame to world frame using rotational matrix
    Fx = R11 * (p1 + d1) + R21 * (p2 + d2) + R31 * (p3 + d3);
    Fy = R12 * (p1 + d1) + R22 * (p2 + d2) + R32 * (p3 + d3);
    Fz = R13 * (p1 + d1) + R23 * (p2 + d2) + R33 * (p3 + d3);

    //Conversion to tau 1, 2, and 3 using jacobian matrix
    tau1_temp = JT_11 * Fx + JT_12 * Fy + JT_13 * Fz + JT_31;
    tau2_temp = JT_21 * Fx + JT_22 * Fy + JT_23 * Fz + JT_32;
    tau3_temp = JT_31 * Fx + JT_32 * Fy + JT_33 * Fz + JT_33;

    /*
     * Implement Friction Compensation
     *
     * Straight line equations for all three joints were given.
     *
     * Viscous friction coefficient is capable of opposing the motion and is proportional to the
     * rotational velocity of the joint.
     *
     * Coulomb friction can be used to calculate the force of dry friction.
     *
     * Positive and negative viscous and coulomb values from the equations were tuned for smooth motion.
     *
     * Lastly, multiply each friction by a friction multiplication factor to minimize friction effects.
     */

    //theta1 friction compensation; set of if else statements that were utilized from lab 3
    if (omega1 > 0.1) {
        u_fric1 = vis_pos_1 * omega1 + cou_pos_1;
    } else if (omega1 < -0.1) {
        u_fric1 = vis_neg_1 * omega1 + cou_neg_1;
    } else {
        u_fric1 = slope1 * omega1;
    }
    tau1_temp += (u_fric1 * ff1);

    //theta2 friction compensation
    if (omega2 > 0.05) {
        u_fric2 = vis_pos_2 * omega2 + cou_pos_2;
    } else if (omega2 < -0.05) {
        u_fric2 = vis_neg_2 * omega2 + cou_neg_2;
    } else {
        u_fric2 = slope2 * omega2;
    }
    tau2_temp += (u_fric2 * ff2);

    //theta3 friction compensation
    if (omega3 > 0.05) {
        u_fric3 = vis_pos_3 * omega3 + cou_pos_3;
    } else if (omega3 < -0.05) {
        u_fric3 = vis_neg_3 * omega3 + cou_neg_3;
    } else {
        u_fric3 = slope3 * omega3;
    }
    tau3_temp += (u_fric3 * ff3);

    /*
     * Saturating motor torque to max of 4.9 and min of -4.9 for safety
     */
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

    //torque definition
    *tau1 = tau1_temp;
    *tau2 = tau2_temp;
    *tau3 = tau3_temp;

    printtheta1motor = theta1motor;
    printtheta2motor = theta2motor;
    printtheta3motor = theta3motor;

    //xyz coordinate of end effector calculation using forward kinematics
    x_loc = (127.0*cos(theta1motor)*(cos(theta3motor) + sin(theta2motor)))/500.0;
    y_loc = (127.0*sin(theta1motor)*(cos(theta3motor) + sin(theta2motor)))/500.0;
    z_loc = (127.0*cos(theta2motor))/500.0 - (127.0*sin(theta3motor))/500.0 + 127.0/500.0;

    //dh theta calculation from xyz coordinate using inverse kinematics
    alpha = atan2(z_loc-L, pow((pow(x_loc,2) + pow(y_loc,2)), 0.5));
    beta = acos((pow(z_loc-L, 2) + pow(x_loc, 2) + pow(y_loc, 2) - 2*pow(L, 2))/(-2*pow(L,2)));
    gamma = (PI - beta) / 2.0;

    theta1 = atan2(y_loc,x_loc);
    theta2 = - (gamma + alpha);
    theta3 = PI - beta;

    //dh theta to motor theta conversion
    mt1 = theta1;
    mt2 = theta2 + PI/2;
    mt3 = theta3 + mt2 - PI/2;

    //Rad to Degree Conversion
    mt1 = mt1*180/PI;
    mt2 = mt2*180/PI;
    mt3 = mt3*180/PI;

    //Simulink plot output variables
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
