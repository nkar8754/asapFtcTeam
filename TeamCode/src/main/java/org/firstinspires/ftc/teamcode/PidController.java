package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.ElapsedTime;

public class PidController{

     private double integralSum = 0;
     private double Kp = 0;
     private double Ki = 0;
     private double Kd = 0;

     private ElapsedTime timer = new ElapsedTime();
     private double lastError = 0;

     public PidController()
     {
          this.integralSum = 0;
          this.Kp = 0;
          this.Ki = 0;
          this.Kd = 0;

          this.timer = new ElapsedTime();
          this.lastError = 0;
     }

     protected double getCorrectValue(double requiredState, double currentState)
     {
          double error = requiredState - currentState;
          integralSum += error * timer.seconds();
          double derivative = (error - lastError) / timer.seconds();
          lastError = error;

          timer.reset();

          return (error * Kp) + (derivative * Kd) + (integralSum * Ki);
     }
}