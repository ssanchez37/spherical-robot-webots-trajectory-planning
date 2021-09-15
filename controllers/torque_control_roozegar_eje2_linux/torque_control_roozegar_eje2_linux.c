/*
 * File:          torque_control.c
 * Date:
 * Description:
 * Author:
 * Modifications:
 */

/*
 * You may need to add include files like <webots/distance_sensor.h> or
 * <webots/motor.h>, etc.
 */
#include <webots/robot.h>
#include <webots/motor.h>
#include <stdio.h>
#include <math.h>
#include <webots/position_sensor.h>
#include <webots/gps.h>
#include <webots/gyro.h>
#include <webots/inertial_unit.h>
#include <webots/display.h>

/*
 * You may want to add macros here.
 */
#define TIME_STEP 64

/*
 * This is the main program.
 * The arguments of the main function can be specified by the
 * "controllerArgs" field of the Robot node
 */
int main(int argc, char **argv) {
  /* necessary to initialize webots stuff */
  wb_robot_init();

  /*
   * You should declare here WbDeviceTag variables for storing
   * robot devices like this:
   *  WbDeviceTag my_sensor = wb_robot_get_device("my_sensor");
   *  WbDeviceTag my_actuator = wb_robot_get_device("my_actuator");
   */
  WbDeviceTag longitudinal = wb_robot_get_device("long_motor");
  wb_motor_set_position(longitudinal, 0.0);
  wb_motor_set_velocity(longitudinal, 0.0);
  wb_motor_enable_torque_feedback(longitudinal, TIME_STEP);
  WbDeviceTag steer = wb_robot_get_device("steering");
  wb_motor_set_position(steer, 0.0);
  wb_motor_set_velocity(steer, 0.0);
  WbDeviceTag my_gps = wb_robot_get_device("my_gps");
  wb_gps_enable(my_gps, TIME_STEP);
  WbDeviceTag long_pos = wb_robot_get_device("long_pos");
  wb_position_sensor_enable(long_pos, TIME_STEP);
  WbDeviceTag inertial = wb_robot_get_device("inertial");
  wb_inertial_unit_enable(inertial, TIME_STEP);
  WbDeviceTag inertialrod = wb_robot_get_device("inertialrod");
  wb_inertial_unit_enable(inertialrod, TIME_STEP);
  WbDeviceTag gyro1 = wb_robot_get_device("gyro1");
  wb_gyro_enable(gyro1, TIME_STEP);
  WbDeviceTag gyroshell = wb_robot_get_device("gyroshell");
  wb_gyro_enable(gyroshell,TIME_STEP);
  WbDeviceTag theta2 = wb_robot_get_device("theta2");
  wb_position_sensor_enable(theta2, TIME_STEP);
  WbDeviceTag display1 = wb_robot_get_device("display_1");
  int d1w = wb_display_get_width(display1);
  int d1h = wb_display_get_height(display1);
  wb_display_set_color(display1, 0X00000000);
  wb_display_fill_rectangle(display1, 0, 0, d1w, d1h);
  WbDeviceTag display2 = wb_robot_get_device("display_2");
  int d2w = wb_display_get_width(display2);
  int d2h = wb_display_get_height(display2);
  wb_display_set_color(display2, 0x00000000);
  wb_display_fill_rectangle(display2, 0, 0, d2w, d2h);
  
  double A = wb_motor_get_acceleration(longitudinal);
  double a = 0;
  double rc = 0;
  double thsteer = 0;
  double Vp = 0;
  double error = 0;
  double previous_error = 0;
  double error_integral = 0;
  double error_derivative = 0;
  double ts = wb_robot_get_basic_time_step()/1000;
  double newVel = 0;
  double maxVel = wb_motor_get_max_torque(longitudinal);
  double the = 0;
  double temp1 = 0;
  double temp2 = 0;
  double thsp = 0;
  double alphadeg = 0;
  double alpha = alphadeg*3.1416/180;
  int i = 1;
  int j = 1;
  double tiempo = 0;
  double betapre = 0;
  double thetapre = 0;
  double th2pre = 0;
  
  double sp = 5;
  double spz = 5;
  double spz2 = -5;
  double r1 = 0.2;
  double r2 = 0.18;
  double m1 = 0.5;
  double m2 = 0.639;
  double ic = 0.05;
  double g = 9.81;
  double phiref = sp/r1;
  double betaref = phiref;
  double rG = m2*r2/(m1+m2);
  double Vd = 1.5;
  
  double posxa = 0;
  /* main loop
   * Perform simulation steps of TIME_STEP milliseconds
   * and leave the loop when the simulation is over
   */
  while (wb_robot_step(TIME_STEP) != -1) {
    /*
     * Read the sensors :
     * Enter here functions to read sensor data, like:
     *  double val = wb_distance_sensor_get_value(my_sensor);
     */
    const double *pos = wb_gps_get_values(my_gps);
    const double *theta = wb_inertial_unit_get_roll_pitch_yaw(inertial);
    const double *theta2 = wb_inertial_unit_get_roll_pitch_yaw(inertialrod);
    const double *thetap = wb_gyro_get_values(gyro1);
    const double *phip = wb_gyro_get_values(gyroshell);
    double beta = wb_position_sensor_get_value(long_pos);
    double th2 = theta[0];//wb_position_sensor_get_value(theta2);
    double phi = beta - theta[1];
    double thetadeg = theta[1]*180/3.1416;
    double betadeg = beta*180/3.1416;
    double phideg = phi*180/3.1416;
    double th2deg = 90-th2*180/3.1416;
    /* Process sensor data here */
    /*
     * Enter here functions to send actuator commands, like:
     * wb_motor_set_position(my_actuator, 10.0);
     */
     
     // Control PID de torque
     
     double P = -43.7707018;//0.15;  // Ganancia proporcional
     double I = -2.9050182;   // Ganancia Integral
     double D = -0.2250206;   // Ganancia Derivativa
     
     double Vd = -4; // rad/s -- Velocidad deseada de la esfera.
     double Vc = wb_motor_get_velocity(longitudinal); // Velocidad actual de la esfera.
     error = Vd - phip[2];
     error_integral = (error_integral + error)*ts;
     error_derivative = (error - previous_error)/ts;
     newVel = P*error + D*error_derivative + I*error_integral;
     printf("DebugPre : %f\n", newVel);
     
     if (newVel > maxVel){
       newVel = maxVel;
     }
     else if (newVel < -maxVel){
       newVel = -maxVel;
     }
     Vp = newVel;
     previous_error = error;
     
     //wb_motor_set_position(longitudinal, INFINITY);
     if (pos[0] <= sp){
       wb_motor_set_torque(longitudinal, newVel);
       wb_motor_set_position(steer, 0);
       wb_motor_set_velocity(steer, 1.5);
       wb_motor_set_control_pid(steer, 1.7, 1.74, 0.01);
     }
     else if (pos[0] > sp){
       wb_motor_set_torque(longitudinal, newVel);
       rc = (spz)/2;
       double thsteer = (((m1+m2)*r1+m2*(r1-r2))*(Vd*Vd*r1*r1)+m2*g*r2*r1)/(m2*g*r2*rc);
       thsteer = (1 - 0.01)*(thsteer - 0.032*thetap[1])+0.01*th2;
       wb_motor_set_position(steer, thsteer);
       wb_motor_set_velocity(steer, 1.5);
       wb_motor_set_control_pid(steer, 1.2, 1.74, 0.01); 
     }
     wb_display_set_color(display1, 0x00FF0000);
     wb_display_draw_line(display1, i-1, (-betapre)+200, i, (-beta)+200);
     wb_display_set_color(display1, 0x0000FF00);
     wb_display_draw_line(display1, i-1, thetapre+100, i, thetadeg+100);
     wb_display_set_color(display2, 0x0000FF00);
     wb_display_draw_line(display2, i-1, th2pre+100, i, th2deg+100);
     i++;
     if (i > d1w){
     i = 0;
     wb_display_set_color(display1, 0x00000000);
     wb_display_fill_rectangle(display1, 0, 0, d1w, d1h);
     wb_display_set_color(display2, 0x00000000);
     wb_display_fill_rectangle(display2, 0, 0, d2w, d2h);
     }
     betapre = beta;
     thetapre = thetadeg;
     th2pre = th2deg;
     printf("Theta1 = %f°\n", thetadeg);
     printf("Theta2 = %f°\n", th2deg);
     printf("Beta = %f°\n", betadeg);
     printf("Phi = %f°\n", phideg);
     printf("X = %fm\n", pos[0]);
     printf("Y = %fm\n", pos[1]);
     printf("Z = %fm\n", pos[2]);
     printf("W = %frad/s\n", thetap[1]);
     printf("Wsph = %frad/s\n", phip[2]);
     printf("Debug = %f\n", tiempo);//the*180/3.1416); //thsteer*180/3.1416);
     printf("Torque M1 = %f\n", wb_motor_get_torque_feedback(longitudinal));
  };

  /* Enter your cleanup code here */

  /* This is necessary to cleanup webots resources */
  wb_robot_cleanup();

  return 0;
}