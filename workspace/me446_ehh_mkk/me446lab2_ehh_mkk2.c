#include "math.h"
#include "F28335Serial.h"

#define PI          3.1415926535897932384626433832795
#define TWOPI       6.283185307179586476925286766559
#define HALFPI      1.5707963267948966192313216916398
#define GRAV        9.81

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

//PI control variables
float Kp1 = 10;
float Kp2 = 10;
float Kp3 = 10;
float KD1 = 1;
float KD2 = 1;
float KD3 = 1;

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

    if ((mycount%2000)==0) {
        if (theta1_des > 0.1) {
            theta1_des = 0;
            theta2_des = 0;
            theta3_des = 0;
        } else {
            theta1_des = PI / 6;
            theta2_des = PI / 6;
            theta3_des = PI / 6;
        }
    }

    //omega1
    omega1 = (theta1motor - theta1_old)/0.001;
    omega1 = (omega1 + omega1_old1 + omega1_old2)/3.0;
    theta1_old = theta1motor;
    omega1_old2 = omega1_old1;
    omega1_old1 = omega1;

    //omega2
    omega2 = (theta2motor - theta2_old)/0.001;
    omega2 = (omega2 + omega2_old2 + omega2_old2)/3.0;
    theta2_old = theta1motor;
    omega2_old2 = omega1_old1;
    omega2_old1 = omega1;

    //omega3
    omega3 = (theta3motor - theta3_old)/0.001;
    omega3 = (omega3 + omega3_old1 + omega3_old2)/3.0;
    theta3_old = theta3motor;
    omega3_old2 = omega3_old1;
    omega3_old1 = omega3;

    tau1_temp = Kp1 * (theta1_des - theta1motor) - KD1 * omega1;
    tau2_temp = Kp2 * (theta2_des - theta2motor) - KD2 * omega2;
    tau3_temp = Kp3 * (theta3_des - theta3motor) - KD3 * omega3;

    //Motor torque limitation(Max: 5 Min: -5)
    if (tau1_temp > 5) {
        tau1_temp = 5;
    } else if (tau1_temp < -5) {
        tau1_temp = -5;
    }

    if (tau2_temp > 5) {
        tau2_temp = 5;
    } else if (tau2_temp < -5) {
        tau2_temp = -5;
    }

    if (tau3_temp > 5) {
        tau3_temp = 5;
    } else if (tau3_temp < -5) {
        tau3_temp = -5;
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
