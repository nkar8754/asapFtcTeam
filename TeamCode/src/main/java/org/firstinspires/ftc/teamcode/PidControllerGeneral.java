
package org.firstinspires.ftc.teamcode;


public class PidControllerGeneral {

     private double integral = 0;
     public double Kp = 0;
     public double Ki = 0;
     public double Kd = 0;

     private double lastError = 0;


     public PidControllerGeneral(double p, double i, double d)
     {
          this.integral = 0;
          this.Kp = p;
          this.Ki = i;
          this.Kd = d;

          this.lastError = 0;
     }
     
     double clamp(double x, double min, double max) { 
          return Math.min(Math.max(x, min), max);
     }

     public double calculate(double targetState, double currentState)
     {
          double error = targetState - currentState;
          integral += error * Ki;
          double derivative = (error - lastError) * Kd;
          lastError = error;

          return error * Kp + derivative + integral;
     }
}

