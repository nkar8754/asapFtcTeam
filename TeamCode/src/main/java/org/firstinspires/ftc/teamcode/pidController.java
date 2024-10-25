package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

public class pidController extends LinearOpMode {

     DcMotor frontLeftMotor;
     DcMotor frontRightMotor;
     DcMotor backLeftMotor;
     DcMotor backRightMotor;

     double integralSum = 0;
     double Kp = 0;
     double Ki = 0;
     double Kd = 0;

     ElapsedTime timer = new ElapsedTime();
     private double lastError = 0;

     @Override
     public void runOpMode() throws InterruptedException {
          getMotor();
          setMotorMode();

          waitForStart();
          while (opModeIsActive()) {
               double power = PIDControl(100, frontLeftMotor.getCurrentPosition());
               frontLeftMotor.setPower(power);
               frontRightMotor.setPower(power);
               backLeftMotor.setPower(power);
               backRightMotor.setPower(power);
          }
     }
     public double PIDControl(double reference, double state) {
          double error = reference - state;
          integralSum += error * timer.seconds();
          double derivative = (error - lastError) / timer.seconds();
          lastError = error;

          timer.reset();

          double output = (error * Kp) + (derivative * Kd) + (integralSum * Ki);
          return output;
     }

     private void getMotor(){
          frontLeftMotor = hardwareMap.get(DcMotor.class, "frontLeftMotor");
          frontRightMotor = hardwareMap.get(DcMotor.class, "frontRightMotor");
          backLeftMotor = hardwareMap.get(DcMotor.class, "backLeftMotor");
          backRightMotor = hardwareMap.get(DcMotor.class, "backRightMotor");
     }

     private void setMotorMode(){
          frontLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
          frontRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
          backLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
          backRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
     }
}