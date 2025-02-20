
package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.util.ElapsedTime;


public class PidController{

     private double integral = 0;
     public double Kp = 0;
     public double Ki = 0;
     public double Kd = 0;

     private ElapsedTime timer = new ElapsedTime();
     private double lastError = 0;


     public PidController(double p, double i, double d)
     {
          this.integral = 0;
          this.Kp = p;
          this.Ki = i;
          this.Kd = d;

          this.timer = new ElapsedTime();
          this.lastError = 0;
     }
     
     double clamp(double x, double min, double max) { 
          return Math.min(Math.max(x, min), max);
     }

     double angleWrap(double difference) {
          while (difference > 0.5) {
               difference -= 1;
          }

          while (difference < -0.5) {
               difference += 1;
          }

          return difference;
     }

     public double calculate(double targetState, double currentState)
     {
          double error = targetState - currentState;
          error = angleWrap(error);
          integral += error * timer.seconds();
          double derivative = (error - lastError) / timer.seconds();
          lastError = error;

          timer.reset();

          return (error * Kp) + (derivative * Kd) + (integral * Ki);
     }
}

