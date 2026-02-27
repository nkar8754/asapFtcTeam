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
public class Auto extends LinearOpMode {
    private SparkFunOTOS odometry;

    private DcMotor frontLeftMotor;
    private DcMotor backLeftMotor;
    private DcMotor frontRightMotor;
    private DcMotor backRightMotor;

    private CRServo frontLeftServo;
    private CRServo backLeftServo;
    private CRServo frontRightServo;
    private CRServo backRightServo;

    public static double kp = 2;
    public static double ki = 0.0;
    public static double kd = 1;

    public static double offsetFR = 90;
    public static double offsetBR = -15;
    public static double offsetFL = 260;
    public static double offsetBL = 155;

    public static double svP = 0;

    AnalogInput backLeftEncoder;
    AnalogInput backRightEncoder;
    AnalogInput frontLeftEncoder;
    AnalogInput frontRightEncoder;

    private PidController pidController1 = null;
    private PidController pidController2 = null;
    private PidController pidController3 = null;
    private PidController pidController4 = null;
    private SwerveKinematics swerveController = new SwerveKinematics(234, 304.812);

    class Robot {
        Pose pose = new Pose(200, 200);
        Pose vel = new Pose();
        Pose acc = new Pose();

        final double DAMP = 0.96;
        final double DAMP_ANGLE = 0.90;

        public void update(Path path) {

            //double d = Math.min(1, Math.pow(pose.distance(path.getLastPoint()) / 160, 2));
            double d = 0.01;

            acc.x *= d;
            acc.y *= d;

            vel.x += acc.x / 6;
            vel.y += acc.y / 6;
            vel.angle += acc.angle / 6;

            pose.x = -odometry.getPosition().y * 100;
            pose.y = odometry.getPosition().x * 100;
            pose.angle += vel.angle;

            vel.x *= DAMP;
            vel.y *= DAMP;
            vel.angle *= DAMP_ANGLE;
        }
    }

    @Override
    public void runOpMode() {
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
        //wrist: 1.01
        //slide: 980
        //inc: 1.616
        ElapsedTime timer = new ElapsedTime();

        Path path = new Path();
        path.addPoint(new PathPoint(0, 31));
        path.addPoint(new PathPoint(-41, -41));
        path.addPoint(new PathPoint(0, -41));
        path.addPoint(new PathPoint(0, 0));
        path.followRadius(200);
        path.constantHeading(Math.PI / 2);

        Robot robot = new Robot();

        while (timer.milliseconds() <= 30000 && opModeIsActive()) {//30500) {
            double rotationRadians = (odometry.getPosition().h * Math.PI) / 180;
            matrix2d referenceTransform = new matrix2d(new ArrayList<Integer>(Arrays.asList(2, 2)));
            referenceTransform.components = new ArrayList<Double>(Arrays.asList(
                    Math.cos(rotationRadians), -Math.sin(rotationRadians),
                    Math.sin(rotationRadians), Math.cos(rotationRadians)
            ));

            robot.update(path);
            path.update(robot.pose);

            Pose follow_pose = path.getFollowPose();
            Circle followCircle = path.getFollowCircle();

            robot.acc.x = Math.cos(robot.pose.angleTo(follow_pose));
            robot.acc.y = Math.sin(robot.pose.angleTo(follow_pose));
            robot.acc.angle = Math.max(Math.min((follow_pose.angle-robot.pose.angle), 0.01), -0.01);

            matrix2d velocityWorld = new matrix2d(new ArrayList<Integer>(Arrays.asList(1, 2)));
            velocityWorld.components = new ArrayList<Double>(Arrays.asList(robot.vel.x * 3.0, robot.vel.y * 3.0));
            velocityWorld = matrix2d.matrixMultiply(referenceTransform, velocityWorld);

            ArrayList<Double> output = swerveController.getVelocities(velocityWorld.components.get(0), velocityWorld.components.get(1), 0);
            drive(output, 1);

            telemetry.addData("posx: ", robot.pose.x);
            telemetry.addData("posy: ", robot.pose.y);
            telemetry.addData("velx: ", robot.vel.x);
            telemetry.addData("vely: ", robot.vel.y);
            telemetry.addData("accx: ", robot.acc.x / 3);
            telemetry.addData("accy: ", robot.acc.y / 3);
            telemetry.addData("followx", follow_pose.x);
            telemetry.addData("followy", follow_pose.y);
            telemetry.addData("distance", follow_pose.distance(path.getLastPoint()));
            telemetry.addData("passed", path.getLastPoint().passed);
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


