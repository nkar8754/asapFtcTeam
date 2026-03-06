package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
//import com.arcrobotics.ftclib.kinematics.wpilibkinematics.ChassisSpeeds;
import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.hardware.motors.Motor;
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
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfInt;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera2;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Scanner;

@Config
@TeleOp
public class SwerveTeleOp extends LinearOpMode {
    OpenCvCamera camera;
    SparkFunOTOS odometry;
    SparkFunOTOS.Pose2D odoPos;

    public static int targetColor = 0;

    static class AnalyzedStone {
        RotatedRect rect;
        double angle;
    }

    private DcMotor frontLeftMotor;
    private DcMotor backLeftMotor;
    private DcMotor frontRightMotor;
    private DcMotor backRightMotor;

    private DcMotorEx topShooter;
    private DcMotorEx bottomShooter;

    private CRServo frontLeftServo;
    private CRServo backLeftServo;
    private CRServo frontRightServo;
    private CRServo backRightServo;

    private Servo topFlap;
    private Servo bottomFlap;
    private CRServo intake;
    private CRServo hood;
    private DcMotor intakeMotor;



    public static double launchVelocity = 5.0;
    public static double kp = 2;
    public static double ki = 0.0;
    public static double kd = 1;
    public static double lkp = 0.005;
    public static double lki = 0.0;
    public static double lkd = 0;
    public static double hkp = 1;
    public static double hkd = 0;

    public static double offsetFR = 90;
    public static double offsetBR = -15;
    public static double offsetFL = 260;
    public static double offsetBL = 155;
    public static double offset = 8.2;

    AnalogInput backLeftEncoder;
    AnalogInput backRightEncoder;
    AnalogInput frontLeftEncoder;
    AnalogInput frontRightEncoder;
    AnalogInput hoodEncoder;

    private PidControllerGeneral hoodPID;
    private PidController pidController1;
    private PidController pidController2;
    private PidController pidController3;
    private PidController pidController4;

    private PidControllerGeneral pidController;

    private SwerveKinematics swerveController = new SwerveKinematics(234, 304.812);

    private double angleDiff(double a, double b) {
        double v1x = Math.cos(a * 2 * Math.PI);
        double v1y = Math.sin(a * 2 * Math.PI);

        double v2x = Math.cos(b * 2 * Math.PI);
        double v2y = Math.sin(b * 2 * Math.PI);

        return v1x * v2x + v1y * v2y;
    }

    private double getLaunchAngle(double posX, double posY, double v) {
        double D = Math.sqrt(Math.pow(110 / 100 - posX, 2) + Math.pow (17.9 / 100 - posY, 2));
        double H = 1.143;
        double A = 9.81;

        double a = (A * Math.pow(D, 2)) / (2 * Math.pow(v, 2));
        double c = a + H;

        double tanTheta = (D + Math.sqrt(Math.pow(D, 2) - 4 * a * c) / (2 * a));

        return Math.atan(tanTheta);
    }

    public static double topFlapKick = 0.55;
    public static double topFlapStow = 0.7;
    public static double bottomFlapStow = 0;
    public static double bottomFlapAgitate = 0.1;

    @Override
    public void runOpMode() {

        topShooter = hardwareMap.get(DcMotorEx.class, "topShooter");
        bottomShooter = hardwareMap.get(DcMotorEx.class, "bottomShooter");
        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
        bottomFlap = hardwareMap.get(Servo.class, "bottomFlap");
        topFlap = hardwareMap.get(Servo.class, "topFlap");
        intake = hardwareMap.get(CRServo.class, "intake");


        topShooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        topShooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        bottomShooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bottomShooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        topShooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bottomShooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeftMotor = hardwareMap.get(DcMotorEx.class, "frontLeftMotor");
        backLeftMotor = hardwareMap.get(DcMotorEx.class, "backLeftMotor");
        frontRightMotor = hardwareMap.get(DcMotorEx.class, "frontRightMotor");
        backRightMotor = hardwareMap.get(DcMotorEx.class, "backRightMotor");

        frontLeftServo = hardwareMap.get(CRServo.class, "frontLeftServo");
        backLeftServo = hardwareMap.get(CRServo.class, "backLeftServo");
        frontRightServo = hardwareMap.get(CRServo.class, "frontRightServo");
        backRightServo = hardwareMap.get(CRServo.class, "backRightServo");

        backLeftEncoder = hardwareMap.get(AnalogInput.class, "backLeftEncoder");
        backRightEncoder = hardwareMap.get(AnalogInput.class, "backRightEncoder");
        frontLeftEncoder = hardwareMap.get(AnalogInput.class, "frontLeftEncoder");
        frontRightEncoder = hardwareMap.get(AnalogInput.class, "frontRightEncoder");

        hood = hardwareMap.get(CRServo.class, "hood");
        hoodEncoder = hardwareMap.get(AnalogInput.class, "hoodEncoder");

        telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry(),telemetry);
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        odometry = hardwareMap.get(SparkFunOTOS.class, "sensor_otos");
        odometry.setLinearUnit(DistanceUnit.METER);
        odometry.setAngularUnit(AngleUnit.DEGREES);
        odometry.setLinearScalar(1.008);
        odometry.setAngularScalar(0.992);
        odometry.calibrateImu();
        odometry.resetTracking();
        SparkFunOTOS.Pose2D currentPosition = new SparkFunOTOS.Pose2D(0, 0, 0);
        odometry.setPosition(currentPosition);
        bottomFlap = hardwareMap.get(Servo.class, "bottomFlap");
        topFlap = hardwareMap.get(Servo.class, "topFlap");

        bottomFlap.setPosition(0.4);
        topFlap.setPosition(0.7);


        waitForStart();

        pidController = new PidControllerGeneral(lkp, lki, lkd);
        pidController1 = new PidController(kp, ki, kd);
        pidController2 = new PidController(kp, ki, kd);
        pidController3 = new PidController(kp, ki, kd);
        pidController4 = new PidController(kp, ki, kd);
        hoodPID = new PidControllerGeneral(hkp, 0, hkd);

        ElapsedTime timer = new ElapsedTime();

        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        topShooter.setVelocity(0);
        bottomShooter.setVelocity(0);

        double previousHoodAngle = (hoodEncoder.getVoltage() / 3.3) * 2 * Math.PI;
        double trueHoodAngle = 0.0;

        while (opModeIsActive()) {

            for (LynxModule hub : allHubs) {
                hub.clearBulkCache();
            }

            double intakePower = 0;
            double hoodAngle = (hoodEncoder.getVoltage() / 3.3) * 2 * Math.PI;

            if (hoodAngle - previousHoodAngle > 1) {
                trueHoodAngle -= 2 * Math.PI;
            }

            if (hoodAngle - previousHoodAngle < -1) {
                trueHoodAngle += 2 * Math.PI;
            }

// RB = intake

            if (gamepad1.x) {
                if (timer.milliseconds() > 0 && timer.milliseconds() <= 1000 && gamepad1.x) {
                    topFlap.setPosition(topFlapKick);
                    bottomFlap.setPosition(bottomFlapStow);
                    intake.setPower(-1);
                    intakeMotor.setPower(-1);
                } else if (timer.milliseconds() > 1000 && timer.milliseconds() <= 1500 && gamepad1.x) {
                    topFlap.setPosition(topFlapStow);
                    intake.setPower(0);
                    intakeMotor.setPower(0);
                } else if (timer.milliseconds() > 1500 && timer.milliseconds() <= 2000 && gamepad1.x) {
                    topFlap.setPosition(topFlapStow);
                    bottomFlap.setPosition(bottomFlapAgitate);
                    intake.setPower(-1);
                    intakeMotor.setPower(-1);
                }
            }

            if (timer.milliseconds() > 2000) {
                timer = new ElapsedTime();
            }

            if (!gamepad1.x) {
                if (gamepad1.a) {
                    bottomFlap.setPosition(bottomFlapAgitate);
                } else if (gamepad1.y) {
                    topFlap.setPosition(topFlapKick);
                }  else  {
                    topFlap.setPosition(topFlapStow);
                    bottomFlap.setPosition(bottomFlapStow);
                }

                if (gamepad1.left_trigger > 0.05) {
                    bottomFlap.setPosition(bottomFlapAgitate);
                    intakePower = -1;
                }
                else if (gamepad1.left_bumper) {
                    intakePower = 1;
                }
                else {
                    intakePower = 0;
                }

                intake.setPower(intakePower);
                intakeMotor.setPower(intakePower);
            }

            telemetry.update();

            pidController.Kp = lkp;
            pidController.Ki = lki;
            pidController.Kd = lkd;
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
            hoodPID.Kp = hkp;
            hoodPID.Kd = hkd;

            double speedMult = 1;

            if (gamepad1.right_bumper ) {
                speedMult = 0.5;
            }

            if (gamepad1.a) {
                SparkFunOTOS.Pose2D pose = odometry.getPosition();
                pose.h = 0;
                odometry.setPosition(pose);
            }

            double shootingAngle = -getLaunchAngle(odometry.getPosition().x, odometry.getPosition().y, launchVelocity) * (136.0 / 24.0) + offset;
            shootingAngle = Math.max(Math.min(shootingAngle, 3.57), 1.0);
            if (shootingAngle != shootingAngle) shootingAngle = 1.0;
            double hoodPower = -2.0 * hoodPID.calculate(shootingAngle / (2 * Math.PI), (trueHoodAngle + hoodAngle) / (2 * Math.PI));
            hood.setPower(hoodPower);

            double rotationRadians = (odometry.getPosition().h * Math.PI) / 180;
            matrix2d referenceTransform = new matrix2d(new ArrayList<Integer>(Arrays.asList(2, 2)));
            referenceTransform.components = new ArrayList<Double>(Arrays.asList(
                    Math.cos(rotationRadians), -Math.sin(rotationRadians),
                    Math.sin(rotationRadians), Math.cos(rotationRadians)
            ));

            matrix2d velocityWorld = new matrix2d(new ArrayList<Integer>(Arrays.asList(1, 2)));
            velocityWorld.components = new ArrayList<Double>(Arrays.asList(-gamepad1.left_stick_y / 1.5, gamepad1.left_stick_x / 1.5));
            velocityWorld = matrix2d.matrixMultiply(referenceTransform, velocityWorld);

            double rot = -gamepad1.right_stick_x / 360;
            if (Math.abs(rot) < 0.0002) rot = 0.0;

            ArrayList<Double> output = swerveController.getVelocities(velocityWorld.components.get(0), velocityWorld.components.get(1), rot);
            drive(output, speedMult);

            telemetry.addData("fr: ", frontRightEncoder.getVoltage() / 3.3);
            telemetry.addData("br: ", backRightEncoder.getVoltage() / 3.3);
            telemetry.addData("fl: ", frontLeftEncoder.getVoltage() / 3.3);
            telemetry.addData("bl: ", backLeftEncoder.getVoltage() / 3.3);
            telemetry.addData("X: ",  -odometry.getPosition().y * 100);
            telemetry.addData("Y: ", odometry.getPosition().x * 100);
            telemetry.addData("rot: ", (odometry.getPosition().h * Math.PI) / 180.0);
            telemetry.addData("hood: ", -(getLaunchAngle(odometry.getPosition().x, odometry.getPosition().y, launchVelocity)) * (136.0 / 24.0));
            telemetry.addData("hoodPower: ", hoodPower);
            telemetry.addData("shooterVelocity: ", topShooter.getVelocity());

// SHOOT
            if (gamepad1.right_trigger > 0.2) {
                double power = pidController.calculate(1100, topShooter.getVelocity());
                topShooter.setPower(power);
                bottomShooter.setPower(power);
                telemetry.addData("shooterPower: ", power);
            } else {
                topShooter.setPower(0);
                bottomShooter.setPower(0);
            }

            telemetry.update();

            previousHoodAngle = hoodAngle;
        }
    }

    private void drive(ArrayList<Double> output, double speedMult) {
        double angleBL = (((output.get(2) / Math.PI) + 1) / 2 + offsetBL / 360) % 1;
        double encoderBL = backLeftEncoder.getVoltage() / 3.3;
        double angleBLOpposite = (angleBL + 0.5) % 1;
        boolean blReverse = angleDiff(encoderBL, angleBL) < 0;
        if (blReverse) angleBL = angleBLOpposite;

        double pid_output1 = -pidController1.calculate(angleBL, encoderBL);
        backLeftServo.setPower(pid_output1 * 2);

        double angleBR = (((output.get(0) / Math.PI) + 1) / 2 + offsetBR / 360) % 1;
        double encoderBR = backRightEncoder.getVoltage() / 3.3;
        double angleBROpposite = (angleBR + 0.5) % 1;
        boolean brReverse = angleDiff(encoderBR, angleBR) < 0;
        if (brReverse) angleBR = angleBROpposite;

        double pid_output2 = -pidController2.calculate(angleBR, encoderBR);
        backRightServo.setPower(pid_output2 * 2);

        double angleFL = (((output.get(4) / Math.PI) + 1) / 2 + offsetFL / 360) % 1;
        double encoderFL = frontLeftEncoder.getVoltage() / 3.3;
        double angleFLOpposite = (angleFL + 0.5) % 1;
        boolean flReverse =  angleDiff(encoderFL, angleFL) < 0;
        if (flReverse) angleFL = angleFLOpposite;

        double pid_output3 = -pidController3.calculate(angleFL, encoderFL);
        frontLeftServo.setPower(pid_output3 * 2);

        double angleFR = (((output.get(6) / Math.PI) + 1) / 2 + offsetFR / 360) % 1;
        double encoderFR = frontRightEncoder.getVoltage() / 3.3;
        double angleFROpposite = (angleFR + 0.5) % 1;
        boolean frReverse = angleDiff(encoderFR, angleFR) < 0;
        if (frReverse) angleFR = angleFROpposite;

        double pid_output4 = -pidController4.calculate(angleFR, encoderFR);
        frontRightServo.setPower(pid_output4 * 2);

        if (Math.abs(pid_output1) < 0.55 && Math.abs(pid_output2) < 0.55 && Math.abs(pid_output3) < 0.55 && Math.abs(pid_output4) < 0.55) {
            frontLeftMotor.setPower(output.get(5) * speedMult * (flReverse ? 1 : -1));
            backLeftMotor.setPower(output.get(3) * speedMult * (blReverse ? 1 : -1));
            frontRightMotor.setPower(output.get(7) * speedMult * (frReverse ? -1 : 1));
            backRightMotor.setPower(output.get(1) * speedMult * (brReverse ? -1 : 1));
        } else {
            frontLeftMotor.setPower(0);
            backLeftMotor.setPower(0);
            frontRightMotor.setPower(0);
            backRightMotor.setPower(0);
        }
    }
}
