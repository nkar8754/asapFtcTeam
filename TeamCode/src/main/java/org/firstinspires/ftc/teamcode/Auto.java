package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;
import static org.firstinspires.ftc.teamcode.RobotMovement.followCurve;
import static org.firstinspires.ftc.teamcode.RobotMovement.goToPosition;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import java.util.ArrayList;
@Autonomous
public class Auto extends LinearOpMode {

    private DcMotor frontLeftMotor;
    private DcMotor backLeftMotor;
    private DcMotor frontRightMotor;
    private DcMotor backRightMotor;
    private DcMotor slideMotor;

    private CRServo frontLeftServo;
    private CRServo backLeftServo;
    private CRServo frontRightServo;
    private CRServo backRightServo;

    public static double kp = 2;
    public static double ki = 1;
    public static double kd = 0.0;

    public static double offsetFR = 130;
    public static double offsetBR = 210;
    public static double offsetFL = 110;
    public static double offsetBL = 20;

    public static double svP = 0;

    AnalogInput backLeftEncoder;
    AnalogInput backRightEncoder;
    AnalogInput frontLeftEncoder;
    AnalogInput frontRightEncoder;

    private PidController pidController5 = new PidController(kp, ki, kd);
    private PidController pidController6 = new PidController(kp, ki, kd);
    private PidController pidController7 = new PidController(kp, ki, kd);
    private PidController pidController8 = new PidController(kp, ki, kd);

    double relativeTurnAngle = getRelativeTurnAngle();
    double moveSpeed = getMoveSpeed();


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

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        telemetry.addData("Status", "Running");
        telemetry.update();

        pidController5.Kp = kp;
        pidController5.Ki = ki;
        pidController5.Kd = kd;
        pidController6.Kp = kp;
        pidController6.Ki = ki;
        pidController6.Kd = kd;
        pidController7.Kp = kp;
        pidController7.Ki = ki;
        pidController7.Kd = kd;
        pidController8.Kp = kp;
        pidController8.Ki = ki;
        pidController8.Kd = kd;

        ArrayList<CurvePoint> allPoints = new ArrayList<>();

        allPoints.add(new CurvePoint(10,10, 1.0, 1.0, 50, Math.toRadians(50), 1.0));
        followCurve(allPoints, Math.toRadians(90));
        goToPosition(10,10, 1.0, 90, 1.0);

        motorOutput();

        allPoints.add(new CurvePoint(0,0, 1.0, 1.0, 50, Math.toRadians(50), 1.0));
        followCurve(allPoints, Math.toRadians(90));

        motorOutput();

        //allPoints.add(new CurvePoint(0,0, 1.0, 1.0, 50, Math.toRadians(50), 1.0));
        //followCurve(allPoints, Math.toRadians(90));

        //motorOutput();

        //allPoints.add(new CurvePoint(0,0, 1.0, 1.0, 50, Math.toRadians(50), 1.0));
        //followCurve(allPoints, Math.toRadians(90));

        //motorOutput();

        //allPoints.add(new CurvePoint(0,0, 1.0, 1.0, 50, Math.toRadians(50), 1.0));
        //followCurve(allPoints, Math.toRadians(90));

        //motorOutput();

}

    public double getRelativeTurnAngle() {
        return relativeTurnAngle;
    }

    public double getMoveSpeed() {
        return moveSpeed;
    }

    public void motorOutput(){
        double pid_output5 = -pidController5.calculate((((relativeTurnAngle / Math.PI) + 1) / 2 + offsetBL / 360) % 1, (backLeftEncoder.getVoltage() / 3.3));
        backLeftServo.setPower(pid_output5 * 2);
        backLeftMotor.setPower(moveSpeed);

        double pid_output6 = -pidController6.calculate((((relativeTurnAngle / Math.PI) + 1) / 2 + offsetBR / 360) % 1, (backRightEncoder.getVoltage() / 3.3));
        backRightServo.setPower(pid_output6 * 2);
        backRightMotor.setPower(moveSpeed);

        double pid_output7 = -pidController7.calculate((((relativeTurnAngle / Math.PI) + 1) / 2 + offsetFL / 360) % 1, (frontLeftEncoder.getVoltage() / 3.3));
        frontLeftServo.setPower(pid_output7 * 2);
        frontLeftMotor.setPower(moveSpeed);

        double pid_output8 = -pidController8.calculate((((relativeTurnAngle / Math.PI) + 1) / 2 + offsetFR / 360) % 1, (frontRightEncoder.getVoltage() / 3.3));
        frontRightServo.setPower(pid_output8 * 2);
        frontRightMotor.setPower(moveSpeed);


    }
}
