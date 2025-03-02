package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.ArrayList;

@Autonomous
public class Auto2 extends LinearOpMode {
    private SparkFunOTOS odometry;

    private DcMotor frontLeftMotor;
    private DcMotor backLeftMotor;
    private DcMotor frontRightMotor;
    private DcMotor backRightMotor;
    private DcMotor slide1;
    private DcMotor slide2;

    private CRServo frontLeftServo;
    private CRServo backLeftServo;
    private CRServo frontRightServo;
    private CRServo backRightServo;

    private CRServo inclination;
    private Servo wrist;
    private Servo claw;

    public static double kp = 2;
    public static double ki = 1;
    public static double kd = 0.0;

    public static double offsetFR = 177;
    public static double offsetBR = -60;
    public static double offsetFL = 130;
    public static double offsetBL = -118;

    public static double svP = 0;

    AnalogInput backLeftEncoder;
    AnalogInput backRightEncoder;
    AnalogInput frontLeftEncoder;
    AnalogInput frontRightEncoder;
    AnalogInput inclinationEncoder;

    private PidController pidController1 = null;
    private PidController pidController2 = null;
    private PidController pidController3 = null;
    private PidController pidController4 = null;
    private GeneralPid inclinationController = null;
    private SwerveKinematics swerveController = new SwerveKinematics(234, 304.812);

    @Override
    public void runOpMode() {
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
        inclinationEncoder = hardwareMap.get(AnalogInput.class, "inclinationEncoder");

        inclination = hardwareMap.get(CRServo.class, "inclination");
        wrist = hardwareMap.get(Servo.class, "wrist");
        claw = hardwareMap.get(Servo.class, "claw");

        odometry = hardwareMap.get(SparkFunOTOS.class, "sensor_otos");
        odometry.setLinearUnit(DistanceUnit.INCH);
        odometry.setAngularUnit(AngleUnit.DEGREES);
        odometry.setLinearScalar(1.008);
        odometry.setAngularScalar(0.992);
        odometry.calibrateImu();
        odometry.resetTracking();
        Odometry.myOtos = this.odometry;
        SparkFunOTOS.Pose2D currentPosition = new SparkFunOTOS.Pose2D(0, 0, 0);
        odometry.setPosition(currentPosition);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        pidController1 = new PidController(kp, ki, kd);
        pidController2 = new PidController(kp, ki, kd);
        pidController3 = new PidController(kp, ki, kd);
        pidController4 = new PidController(kp, ki, kd);
        inclinationController = new GeneralPid(2, 0, 0);

        telemetry.addData("Status", "Running");
        telemetry.update();

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
        //wrist: 1.01
        //slide: 980
        //inc: 1.616
        ElapsedTime timer = new ElapsedTime();

        while (timer.milliseconds() <= 1500) {
            ArrayList<Double> output = swerveController.getVelocities(0.5,0, 0);
            drive(output, 1);
        }

        ArrayList<Double> output = swerveController.getVelocities(0,0, 0);
        drive(output, 1);
    }

    private void drive(ArrayList<Double> output, double speedMult) {
        double pid_output1 = -pidController1.calculate((((output.get(2) / Math.PI) + 1) / 2 + offsetBL / 360) % 1, (backLeftEncoder.getVoltage() / 3.3));
        backLeftServo.setPower(pid_output1 * 2);

        double pid_output2 = -pidController2.calculate((((output.get(0) / Math.PI) + 1) / 2 + offsetBR / 360) % 1, (backRightEncoder.getVoltage() / 3.3));
        backRightServo.setPower(pid_output2 * 2);

        double pid_output3 = -pidController3.calculate((((output.get(4) / Math.PI) + 1) / 2 + offsetFL / 360) % 1, (frontLeftEncoder.getVoltage() / 3.3));
        frontLeftServo.setPower(pid_output3 * 2);

        double pid_output4 = -pidController4.calculate((((output.get(6) / Math.PI) + 1) / 2 + offsetFR / 360) % 1, (frontRightEncoder.getVoltage() / 3.3));
        frontRightServo.setPower(pid_output4 * 2);

        if (Math.abs(pid_output1) < 0.1 && Math.abs(pid_output2) < 0.1 && Math.abs(pid_output3) < 0.1 && Math.abs(pid_output4) < 0.1) {
            frontLeftMotor.setPower(output.get(5) * speedMult);
            backLeftMotor.setPower(output.get(3) * speedMult);
            frontRightMotor.setPower(output.get(7) * speedMult);
            backRightMotor.setPower(output.get(1) * speedMult);
        } else {
            frontLeftMotor.setPower(0);
            backLeftMotor.setPower(0);
            frontRightMotor.setPower(0);
            backRightMotor.setPower(0);
        }
    }
}
