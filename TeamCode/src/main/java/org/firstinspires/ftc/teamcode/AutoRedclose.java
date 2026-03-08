package org.firstinspires.ftc.teamcode;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import com.qualcomm.hardware.lynx.LynxModule;
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
import org.firstinspires.ftc.teamcode.path.Path;
import org.firstinspires.ftc.teamcode.path.PathPoint;
import org.firstinspires.ftc.teamcode.geometry.Pose;
import org.firstinspires.ftc.teamcode.geometry.Circle;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.ArrayList;

@Autonomous
public class AutoRedclose extends LinearOpMode {
    private SparkFunOTOS odometry;

    private DcMotor frontLeftMotor;
    private DcMotor backLeftMotor;
    private DcMotor frontRightMotor;
    private DcMotor backRightMotor;

    private CRServo frontLeftServo;
    private CRServo backLeftServo;
    private CRServo frontRightServo;
    private CRServo backRightServo;
    private DcMotorEx topShooter;
    private DcMotorEx bottomShooter;
    private Servo topFlap;
    private Servo bottomFlap;
    private CRServo intake;
    private DcMotor intakeMotor;

    public static double kp = 2;
    public static double ki = 0.0;
    public static double kd = 1;
    public static double lkp = 0.005;
    public static double lki = 0.0;
    public static double lkd = 0;

    public static double offsetFR = 90;
    public static double offsetBR = -15;
    public static double offsetFL = 260;
    public static double offsetBL = 155;

    public static double svP = 0;

    private ElapsedTime flapTimer = new ElapsedTime();
    private ElapsedTime shootTimer = new ElapsedTime();
    private boolean flapped = false;
    private boolean staged = false;
    private int ballsShot = 0;

    AnalogInput backLeftEncoder;
    AnalogInput backRightEncoder;
    AnalogInput frontLeftEncoder;
    AnalogInput frontRightEncoder;

    private PidController pidController1 = null;
    private PidController pidController2 = null;
    private PidController pidController3 = null;
    private PidController pidController4 = null;
    private PidController rotator = null;
    private PidControllerGeneral pidController;
    private SwerveKinematics swerveController = new SwerveKinematics(234, 304.812);

    class Robot {
        public Pose pose = new Pose(0, 0);
        public Pose vel = new Pose();
        public Pose acc = new Pose();

        final double DAMP = 0.86;
        final double DAMP_ANGLE = 0.7;

        public SparkFunOTOS.Pose2D update(Path path) {
            SparkFunOTOS.Pose2D pose2d = odometry.getPosition();
            double d = Math.min(1, Math.pow(pose.distance(path.getLastPoint()) / 160, 2));

            acc.x *= d;
            acc.y *= d;

            vel.x += acc.x / 6;
            vel.y += acc.y / 6;
            vel.angle += acc.angle / 7;

            pose.x = -pose2d.y * 100;
            pose.y = pose2d.x * 100;
            pose.angle = (pose2d.h * Math.PI) / 180.0;

            vel.x *= DAMP;
            vel.y *= DAMP;
            vel.angle *= DAMP_ANGLE;

            double length = Math.sqrt(vel.x * vel.x + vel.y * vel.y);
            Pose velDir = new Pose(vel.x / length, vel.y / length);

            if (length > 0.1) {
                vel.x = velDir.x * 0.1;
                vel.y = velDir.y * 0.1;
            }

            return pose2d;
        }
    }

    @Override
    public void runOpMode() {
        topShooter = hardwareMap.get(DcMotorEx.class, "topShooter");
        bottomShooter = hardwareMap.get(DcMotorEx.class, "bottomShooter");
        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
        bottomFlap = hardwareMap.get(Servo.class, "bottomFlap");
        topFlap = hardwareMap.get(Servo.class, "topFlap");
        intake = hardwareMap.get(CRServo.class, "intake");

        frontLeftMotor = hardwareMap.get(DcMotorEx.class, "frontLeftMotor");
        backLeftMotor = hardwareMap.get(DcMotorEx.class, "backLeftMotor");
        frontRightMotor = hardwareMap.get(DcMotorEx.class, "frontRightMotor");
        backRightMotor = hardwareMap.get(DcMotorEx.class, "backRightMotor");

        backLeftEncoder = hardwareMap.get(AnalogInput.class, "backLeftEncoder");
        backRightEncoder = hardwareMap.get(AnalogInput.class, "backRightEncoder");
        frontLeftEncoder = hardwareMap.get(AnalogInput.class, "frontLeftEncoder");
        frontRightEncoder = hardwareMap.get(AnalogInput.class, "frontRightEncoder");

        frontLeftServo = hardwareMap.get(CRServo.class, "frontLeftServo");
        backLeftServo = hardwareMap.get(CRServo.class, "backLeftServo");
        frontRightServo = hardwareMap.get(CRServo.class, "frontRightServo");
        backRightServo = hardwareMap.get(CRServo.class, "backRightServo");

        topShooter.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        topShooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        bottomShooter.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        bottomShooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        odometry = hardwareMap.get(SparkFunOTOS.class, "sensor_otos");
        odometry.setLinearUnit(DistanceUnit.METER);
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

        pidController = new PidControllerGeneral(lkp, lki, lkd);
        pidController1 = new PidController(kp, ki, kd);
        pidController2 = new PidController(kp, ki, kd);
        pidController3 = new PidController(kp, ki, kd);
        pidController4 = new PidController(kp, ki, kd);

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

        ElapsedTime timer = new ElapsedTime();
        ElapsedTime shooterTimer = new ElapsedTime();

        Path path = new Path();
        path.addPoint(new PathPoint(0, 0));
        path.addPoint(new PathPoint(-0.2747, -47.9126));
        path.followRadius(3);
        path.constantHeading(0);

        Robot robot = new Robot();

        boolean notFiring = true;

        double topFlapKick = 0.55;
        double topFlapStow = 0.7;
        double bottomFlapStow = 0;
        double bottomFlapAgitate = 0.1;

        topShooter.setPower(0);
        bottomShooter.setPower(0);

        ElapsedTime loopTimer = new ElapsedTime();

        while (timer.milliseconds() <= 30000 && opModeIsActive()) {//30500) {
            loopTimer.reset();
            SparkFunOTOS.Pose2D pose2d = robot.update(path);
            path.update(robot.pose);

            Pose follow_pose = path.getFollowPose();
            Circle followCircle = path.getFollowCircle();

            robot.acc.x = Math.cos(robot.pose.angleTo(follow_pose));
            robot.acc.y = Math.sin(robot.pose.angleTo(follow_pose));
            robot.acc.angle = Math.max(Math.min((follow_pose.angle-robot.pose.angle), 0.01), -0.01);

            double rotationRadians = (pose2d.h * Math.PI) / 180.0;
            matrix2d referenceTransform = new matrix2d(new ArrayList<Integer>(Arrays.asList(2, 2)));
            referenceTransform.components = new ArrayList<Double>(Arrays.asList(
                    Math.cos(rotationRadians), -Math.sin(rotationRadians),
                    Math.sin(rotationRadians), Math.cos(rotationRadians)
            ));

            matrix2d velocityWorld = new matrix2d(new ArrayList<Integer>(Arrays.asList(1, 2)));
            velocityWorld.components = new ArrayList<Double>(Arrays.asList(robot.vel.y * 5, robot.vel.x * 5));
            velocityWorld = matrix2d.matrixMultiply(referenceTransform, velocityWorld);

            ArrayList<Double> output = swerveController.getVelocities(velocityWorld.components.get(0), velocityWorld.components.get(1), robot.vel.angle / 12);
            drive(output, 1);

            if (Math.sqrt(robot.vel.x * robot.vel.x + robot.vel.y * robot.vel.y) < 0.05 && timer.milliseconds() > 2000) {
                if (notFiring) {
                    shooterTimer = new ElapsedTime();
                    notFiring = false;
                }

                double velocity = topShooter.getVelocity();
                double power = pidController.calculate(1100, velocity);
                topShooter.setPower(power);
                bottomShooter.setPower(power);
                telemetry.addData("timer: ", shooterTimer.milliseconds());
                telemetry.update();

                if (shooterTimer.milliseconds() > 3000 && shooterTimer.milliseconds() <= 4000) {
                    topFlap.setPosition(topFlapStow);
                    bottomFlap.setPosition(bottomFlapAgitate);
                    intake.setPower(-1);
                    intakeMotor.setPower(-1);
                } else if (shooterTimer.milliseconds() > 4000 && shooterTimer.milliseconds() <= 4500) {
                    topFlap.setPosition(topFlapKick);
                    intake.setPower(-0.0);
                    intakeMotor.setPower(-0.0);
                } else if (shooterTimer.milliseconds() > 4500 && shooterTimer.milliseconds() <= 5000) {
                    topFlap.setPosition(topFlapStow);
                    bottomFlap.setPosition(bottomFlapAgitate);
                    intake.setPower(-1);
                    intakeMotor.setPower(-1);
                } else if (shooterTimer.milliseconds() > 5000 && shooterTimer.milliseconds() <= 5500) {
                    intake.setPower(-1.0);
                    intakeMotor.setPower(-1.0);
                } else if (shooterTimer.milliseconds() > 5000 && shooterTimer.milliseconds() <= 6000) {
                    topFlap.setPosition(topFlapKick);
                    bottomFlap.setPosition(bottomFlapAgitate);
                    intake.setPower(-1);
                    intakeMotor.setPower(-1);
                } else if (shooterTimer.milliseconds() > 6000) {
                    break;
                }
            } else {
                bottomFlap.setPosition(bottomFlapStow);
                topFlap.setPosition(topFlapStow);
                intake.setPower(0);
                intakeMotor.setPower(0);
            }
        }

        bottomFlap.setPosition(bottomFlapStow);
        topFlap.setPosition(topFlapStow);
        intake.setPower(0);
        intakeMotor.setPower(0);

        topShooter.setPower(0);
        bottomShooter.setPower(0);

        path = null;
        path = new Path();
        path.addPoint(new PathPoint(-0.2747, -47.9126));
        path.addPoint(new PathPoint(64.3115, -33.334));
        path.followRadius(5);
        path.constantHeading(-0.644);

        robot = null;
        robot = new Robot();

        ElapsedTime timer2 = new ElapsedTime();

        while (timer.milliseconds() <= 30000 && opModeIsActive()) {//30500) {
            SparkFunOTOS.Pose2D pose2d = robot.update(path);
            path.update(robot.pose);

            Pose follow_pose = path.getFollowPose();
            Circle followCircle = path.getFollowCircle();

            robot.acc.x = Math.cos(robot.pose.angleTo(follow_pose));
            robot.acc.y = Math.sin(robot.pose.angleTo(follow_pose));
            robot.acc.angle = Math.max(Math.min((follow_pose.angle-robot.pose.angle), 0.01), -0.01);

            double rotationRadians = (pose2d.h * Math.PI) / 180.0;
            matrix2d referenceTransform = new matrix2d(new ArrayList<Integer>(Arrays.asList(2, 2)));
            referenceTransform.components = new ArrayList<Double>(Arrays.asList(
                    Math.cos(rotationRadians), -Math.sin(rotationRadians),
                    Math.sin(rotationRadians), Math.cos(rotationRadians)
            ));

            matrix2d velocityWorld = new matrix2d(new ArrayList<Integer>(Arrays.asList(1, 2)));
            velocityWorld.components = new ArrayList<Double>(Arrays.asList(robot.vel.y * 5.5, robot.vel.x * 5.5));
            velocityWorld = matrix2d.matrixMultiply(referenceTransform, velocityWorld);

            ArrayList<Double> output = swerveController.getVelocities(velocityWorld.components.get(0), velocityWorld.components.get(1), robot.vel.angle / 7);
            drive(output, 1);

            if (Math.sqrt(robot.vel.x * robot.vel.x + robot.vel.y * robot.vel.y) < 0.03 && timer2.milliseconds() > 2000) {
                break;
            }

            telemetry.addData("posx: ", path.robot_pose.x);
            telemetry.addData("posy: ", path.robot_pose.y);

            telemetry.addData("velx: ", robot.vel.x * 4);
            telemetry.addData("vely: ", robot.vel.y * 4);
            telemetry.addData("angular: ", robot.vel.angle);
            telemetry.addData("accx: ", robot.acc.x / 3);
            telemetry.addData("accy: ", robot.acc.y / 3);
            telemetry.addData("followx: ", follow_pose.x);
            telemetry.addData("followy: ", follow_pose.y);
            telemetry.addData("distance: ", robot.pose.distance(path.getLastPoint()));
            telemetry.addData("passed: ", path.getLastPoint().passed);
            telemetry.update();
        }

        ArrayList<Double> output = swerveController.getVelocities(0,0, 0);
        drive(output, 1);
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

    private double angleDiff(double a, double b) {
        double v1x = Math.cos(a * 2 * Math.PI);
        double v1y = Math.sin(a * 2 * Math.PI);

        double v2x = Math.cos(b * 2 * Math.PI);
        double v2y = Math.sin(b * 2 * Math.PI);

        return v1x * v2x + v1y * v2y;
    }
}


