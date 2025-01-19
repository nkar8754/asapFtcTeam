package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
//import com.arcrobotics.ftclib.kinematics.wpilibkinematics.ChassisSpeeds;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
//import com.arcrobotics.ftclib.kinematics.wpilibkinematics.SwerveDriveKinematics;
//import com.arcrobotics.ftclib.kinematics.wpilibkinematics.SwerveModuleState;
//import com.arcrobotics.ftclib.geometry.Translation2d;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.internal.usb.EthernetOverUsbSerialNumber;

import java.util.ArrayList;
import java.util.List;
import java.util.Scanner;

@Config
@TeleOp
public class SwerveTeleOp extends LinearOpMode {
    private DcMotor frontLeftMotor;
    private DcMotor backLeftMotor;
    private DcMotor frontRightMotor;
    private DcMotor backRightMotor;
    private DcMotor slideMotor;

    private CRServo frontLeftServo;
    private CRServo backLeftServo;
    private CRServo frontRightServo;
    private CRServo backRightServo;

    private Servo clawRotator;
    private Servo armRotator;
    private Servo clawActuator;

    public static double kp = 2;
    public static double ki = 0.5;
    public static double kd = 0.0;

    public static double offsetFR = -40;
    public static double offsetBR = -60;
    public static double offsetFL = 100;
    public static double offsetBL = -10;

    public static double svP = 0;

    AnalogInput backLeftEncoder;
    AnalogInput backRightEncoder;
    AnalogInput frontLeftEncoder;
    AnalogInput frontRightEncoder;

    private PidController pidController1 = new PidController(kp, ki, kd);
    private PidController pidController2 = new PidController(kp, ki, kd);
    private PidController pidController3 = new PidController(kp, ki, kd);
    private PidController pidController4 = new PidController(kp, ki, kd);

    private SwerveKinematics swerveController = new SwerveKinematics(234, 304.812);

    double clawAngle = 0.0;
    double armAngle = 0.0;
    double clawActuation = 0.0;
    int slideMotorPosition = 0;

    @Override
    public void runOpMode() {
        frontLeftMotor = hardwareMap.get(DcMotorEx.class, "frontLeftMotor");
        backLeftMotor = hardwareMap.get(DcMotorEx.class, "backLeftMotor");
        frontRightMotor = hardwareMap.get(DcMotorEx.class, "frontRightMotor");
        backRightMotor = hardwareMap.get(DcMotorEx.class, "backRightMotor");
        slideMotor = hardwareMap.get(DcMotorEx.class, "slideMotor");
        slideMotor.setTargetPosition(slideMotorPosition);
        slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slideMotor.setPower(0.6);

        frontLeftServo = hardwareMap.get(CRServo.class, "frontLeftServo");
        backLeftServo = hardwareMap.get(CRServo.class, "backLeftServo");
        frontRightServo = hardwareMap.get(CRServo.class, "frontRightServo");
        backRightServo = hardwareMap.get(CRServo.class, "backRightServo");

        backLeftEncoder = hardwareMap.get(AnalogInput.class, "backLeftEncoder");
        backRightEncoder = hardwareMap.get(AnalogInput.class, "backRightEncoder");
        frontLeftEncoder = hardwareMap.get(AnalogInput.class, "frontLeftEncoder");
        frontRightEncoder = hardwareMap.get(AnalogInput.class, "frontRightEncoder");

        clawRotator = hardwareMap.get(Servo.class, "clawRotator");
        armRotator = hardwareMap.get(Servo.class, "armRotator");
        clawActuator = hardwareMap.get(Servo.class, "clawActuator");

        telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry(),telemetry);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        telemetry.addData("Status", "Running");
        telemetry.update();

        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        while (opModeIsActive()) {
            for (LynxModule hub : allHubs) {
                hub.clearBulkCache();
            }

            pidController1.Kp = kp;
            pidController1.Ki = ki;
            pidController1.Kd = kd;
            pidController2.Kp = kp;
            pidController2.Ki = ki;
            pidController2.Kd = kd;
            pidController3.Kp = kp;
            pidController3.Ki = ki;
            pidController3.Kd = kd;
            pidController4.Kp = kp;
            pidController4.Ki = ki;
            pidController4.Kd = kd;

            ArrayList<Double> output = swerveController.getVelocities(-gamepad1.left_stick_y / 1.5, gamepad1.left_stick_x / 1.5, -gamepad1.right_stick_x / 360);

            double pid_output1 = -pidController1.calculate((((output.get(2) / Math.PI) + 1) / 2 + offsetBL / 360) % 1, (backLeftEncoder.getVoltage() / 3.3));
            backLeftServo.setPower(pid_output1 * 2);

            double pid_output2 = -pidController2.calculate((((output.get(0) / Math.PI) + 1) / 2 + offsetBR / 360) % 1, (backRightEncoder.getVoltage() / 3.3));
            backRightServo.setPower(pid_output2 * 2);

            double pid_output3 = -pidController3.calculate((((output.get(4) / Math.PI) + 1) / 2 + offsetFL / 360) % 1, (frontLeftEncoder.getVoltage() / 3.3));
            frontLeftServo.setPower(pid_output3 * 2);

            double pid_output4 = -pidController4.calculate((((output.get(6) / Math.PI) + 1) / 2 + offsetFR / 360) % 1, (frontRightEncoder.getVoltage() / 3.3));
            frontRightServo.setPower(pid_output4 * 2);

            if (Math.abs(pid_output1) < 0.3 && Math.abs(pid_output2) < 0.3 && Math.abs(pid_output3) < 0.3 && Math.abs(pid_output4) < 0.3) {
                frontLeftMotor.setPower(output.get(5));
                backLeftMotor.setPower(output.get(3));
                frontRightMotor.setPower(output.get(7));
                backRightMotor.setPower(output.get(1));
            } else {
                frontLeftMotor.setPower(0);
                backLeftMotor.setPower(0);
                frontRightMotor.setPower(0);
                backRightMotor.setPower(0);
            }

            if (gamepad1.y && armAngle <= 1.0) {
                armAngle += 0.01;
            } else if(gamepad1.a && armAngle >= 0) {
                armAngle -= 0.01;
            }

            if (gamepad1.x && clawAngle <= 1.0) {
                clawAngle += 0.01;
            } else if(gamepad1.b && clawAngle >= 0) {
                clawAngle -= 0.01;
            }

            if (gamepad1.right_bumper && clawActuation <= 1.0) {
                clawActuation += 0.01;
            } else if(gamepad1.left_bumper && clawActuation >= 0) {
                clawActuation -= 0.01;
            }

            if (gamepad1.dpad_down) {
                slideMotorPosition += 25;
            } else if (gamepad1.dpad_up) {
                slideMotorPosition -= 25;
            }

            armRotator.setPosition(armAngle);
            clawRotator.setPosition(clawAngle);
            clawActuator.setPosition(clawActuation);
            slideMotor.setTargetPosition(slideMotorPosition);

            telemetry.addData("Servo Power", pid_output1);
            telemetry.addData("EncoderBR", backRightEncoder.getVoltage() / 3.3);
            telemetry.addData("EncoderBL", backLeftEncoder.getVoltage() / 3.3);
            telemetry.addData("EncoderFR", frontRightEncoder.getVoltage() / 3.3);
            telemetry.addData("EncoderFL", frontLeftEncoder.getVoltage() / 3.3);

            telemetry.addData("w1", ((output.get(4) / Math.PI + 1) / 2));
            telemetry.addData("w2", ((output.get(6) / Math.PI + 1) / 2));
            telemetry.addData("w3", ((output.get(2) / Math.PI + 1) / 2));
            telemetry.addData("w4", ((output.get(0) / Math.PI + 1) / 2));

            telemetry.addData("Status", "Running");
            telemetry.update();
        }
    }
}
