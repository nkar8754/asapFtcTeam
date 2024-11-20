package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
//import com.arcrobotics.ftclib.kinematics.wpilibkinematics.ChassisSpeeds;
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

import java.util.List;
import java.security.InvalidParameterException;

@Config
@TeleOp
public class SwerveTeleOp extends LinearOpMode {
    private DcMotor frontLeftMotor;
    private DcMotor backLeftMotor;
    private DcMotor frontRightMotor;
    private DcMotor backRightMotor;

    private CRServo frontLeftServo;
    private CRServo backLeftServo;
    private CRServo frontRightServo;
    private CRServo backRightServo;

    public static double kp = 2;
    public static double ki = 0.6;
    public static double kd = 0.04;

//    public static double Prop1 = 0;
//    public static double Int1 = 0;
//    public static double Deriv1 = 0;
//
//    public static double Prop2 = 0;
//    public static double Int2 = 0;
//    public static double Deriv2 = 0;
//
//    public static double Prop3 = 0;
//    public static double Int3 = 0;
//    public static double Deriv3 = 0;
//
//    public static double Prop4 = 0;
//    public static double Int4 = 0;
//    public static double Deriv4 = 0;
//
//    public static double svP = 0;

    public static double offsetFR = 100;
    public static double offsetBR = 130;
    public static double offsetFL = -100;
    public static double offsetBL = -180;

    AnalogInput backLeftEncoder;
    AnalogInput backRightEncoder;
    AnalogInput frontLeftEncoder;
    AnalogInput frontRightEncoder;

    private PidController pidControllerBL = new PidController(kp, ki, kd);
    private PidController pidControllerBR = new PidController(kp, ki, kd);
    private PidController pidControllerFL = new PidController(kp, ki, kd);
    private PidController pidControllerFR = new PidController(kp, ki, kd);

    private enum ServoPositionEnum {
        BackLeft,
        BackRight,
        FrontLeft,
        FrontRight
    }

    // run until the end of the match (driver presses STOP)
    private double tgtPower = 0;
    private double servoPower = 0;

    private double totalRotationBL = 0;
    private double totalRotationBR = 0;
    private double totalRotationFL = 0;
    private double totalRotationFR = 0;

    private int revolutionCountBL = 0;
    private int revolutionCountBR = 0;
    private int revolutionCountFL = 0;
    private int revolutionCountFR = 0;

    private double previousEncoderBL = 0;
    private double previousEncoderBR = 0;
    private double previousEncoderFL = 0;
    private double previousEncoderFR = 0;

    //private SwerveKinematics SwerveMove = new SwerveKinematics(1,2,3,16,16);

    @Override
    public void runOpMode() {

        initializeMotors();

        initializeServos();

        initializeEncoders();

        telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry(),telemetry);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        telemetry.addData("Status", "Running");
        telemetry.update();

        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        for(LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        while (opModeIsActive()) {
            for(LynxModule hub : allHubs) {
                hub.clearBulkCache();
            }
            pidControllerBL.Kp = kp;
            pidControllerBL.Ki = ki;
            pidControllerBL.Kd = kd;
            pidControllerBR.Kp = kp;
            pidControllerBR.Ki = ki;
            pidControllerBR.Kd = kd;
            pidControllerFL.Kp = kp;
            pidControllerFL.Ki = ki;
            pidControllerFL.Kd = kd;
            pidControllerFR.Kp = kp;
            pidControllerFR.Ki = ki;
            pidControllerFR.Kd = kd;

            if (Math.abs(this.gamepad1.left_stick_y) > 0.1) {
                tgtPower = -this.gamepad1.left_stick_y;
            } else {
                tgtPower = 0;
            }

            if (Math.abs(this.gamepad1.right_stick_y) > 0.1) {
                servoPower = -this.gamepad1.right_stick_y;
            } else {
                servoPower = 0;
            }

            frontLeftMotor.setPower(-tgtPower);
            backLeftMotor.setPower(-tgtPower);
            frontRightMotor.setPower(-tgtPower);
            backRightMotor.setPower(-tgtPower);

            setServoPower(ServoPositionEnum.BackLeft, true);
            setServoPower(ServoPositionEnum.BackRight, false);
            setServoPower(ServoPositionEnum.FrontLeft, false);
            setServoPower(ServoPositionEnum.FrontRight, false);

            telemetry.addData("Motor Power", tgtPower);
            telemetry.addData("Status", "Running");
            telemetry.update();
        }
    }

    private void initializeMotors(){
        frontLeftMotor = hardwareMap.get(DcMotorEx.class, "frontLeftMotor");
        backLeftMotor = hardwareMap.get(DcMotorEx.class, "backLeftMotor");
        frontRightMotor = hardwareMap.get(DcMotorEx.class, "frontRightMotor");
        backRightMotor = hardwareMap.get(DcMotorEx.class, "backRightMotor");
    }

    private void initializeServos(){
        frontLeftServo = hardwareMap.get(CRServo.class, "frontLeftServo");
        backLeftServo = hardwareMap.get(CRServo.class, "backLeftServo");
        frontRightServo = hardwareMap.get(CRServo.class, "frontRightServo");
        backRightServo = hardwareMap.get(CRServo.class, "backRightServo");
    }

    private void initializeEncoders(){
        backLeftEncoder = hardwareMap.get(AnalogInput.class, "backLeftEncoder");
        backRightEncoder = hardwareMap.get(AnalogInput.class, "backRightEncoder");
        frontLeftEncoder = hardwareMap.get(AnalogInput.class, "frontLeftEncoder");
        frontRightEncoder = hardwareMap.get(AnalogInput.class, "frontRightEncoder");
    }

    /*
    Sets the servo power by position value.
     */
    private void setServoPower(ServoPositionEnum servoPosition, boolean displayServoPowerTelemetry)
    {
        ServoParameters servoParams = getServoParameters(servoPosition);

        PidController pidController = servoParams.getPidController();
        AnalogInput encoder = servoParams.getEncoder();
        CRServo servo = servoParams.getServo();
        double encoderInDeg = servoParams.getEncoderInDeg();
        double prevEncoderInDeg = servoParams.getPreviousEncoderInDeg();
        int revolutionCount = servoParams.getRevolutionCount();
        double offset = servoParams.getOffset();

        double deltaAngle = Math.abs(encoderInDeg) - Math.abs(prevEncoderInDeg);
        if (deltaAngle < -180 && encoderInDeg < 180) {
            revolutionCount += 1;
        } else if (deltaAngle > 180 && encoderInDeg > 180) {
            revolutionCount -= 1;
        }

        double totalRotation = (encoderInDeg + revolutionCount * 360 - offset);

        setPreviousEncoderInDeg(servoPosition, encoderInDeg);
        setRevolutionCount(servoPosition, revolutionCount);

        double pid_output = -pidController.calculate(servoPower, (totalRotation / 360.0) * 2.0 - 1.0);
        servo.setPower(pid_output * 2);

        if (displayServoPowerTelemetry) {
            telemetry.addData("Servo Power", pid_output);
        }
        String encoderRotationCaption = String.format("Encoder%s", servoPosition.name());
        telemetry.addData(encoderRotationCaption, totalRotation);

        String revolutionsCaption = String.format("Revolutions%s", servoPosition.name());
        telemetry.addData(revolutionsCaption, revolutionCount * 100);
    }

    /*
    Returns the pidController based on the position value.
     */
    private ServoParameters getServoParameters(ServoPositionEnum servoPosition){
        switch (servoPosition)
        {
            case BackLeft:
                return new ServoParameters(
                        pidControllerBL,
                        backLeftServo,
                        backLeftEncoder,
                        previousEncoderBL,
                        revolutionCountBL,
                        offsetBL);
            case BackRight:
                return new ServoParameters(
                        pidControllerBR,
                        backRightServo,
                        backRightEncoder,
                        previousEncoderBR,
                        revolutionCountBR,
                        offsetBR);
            case FrontLeft:
                return new ServoParameters(
                        pidControllerFL,
                        frontLeftServo,
                        frontLeftEncoder,
                        previousEncoderFL,
                        revolutionCountFL,
                        offsetFL);
            case FrontRight:
                return new ServoParameters(
                        pidControllerFR,
                        frontRightServo,
                        frontRightEncoder,
                        previousEncoderFR,
                        revolutionCountFR,
                        offsetFR);
            default:
                throw new InvalidParameterException("Invalid servo motor position");
        }
    }

    /*
    Sets revolution count based on the position value.
     */
    private void setRevolutionCount(ServoPositionEnum servoPosition, int newValue){
        switch (servoPosition)
        {
            case BackLeft:
                revolutionCountBL = newValue;
            case BackRight:
                revolutionCountBR = newValue;
            case FrontLeft:
                revolutionCountFL = newValue;
            case FrontRight:
                revolutionCountFR = newValue;
            default:
                throw new InvalidParameterException("Invalid servo motor position");
        }
    }

    /*
    Sets previous encoder value based on the position value.
     */
    private void setPreviousEncoderInDeg(ServoPositionEnum servoPosition, double newValue){
        switch (servoPosition)
        {
            case BackLeft:
                previousEncoderBL = newValue;
            case BackRight:
                previousEncoderBR = newValue;
            case FrontLeft:
                previousEncoderFL = newValue;
            case FrontRight:
                previousEncoderFR = newValue;
            default:
                throw new InvalidParameterException("Invalid servo motor position");
        }
    }

//    /*
//    Returns the encoder based on the position value.
//     */
//    private AnalogInput getPositionEncoder(ServoPositionEnum servoPosition){
//        switch (servoPosition)
//        {
//            case BackLeft:
//                return backLeftEncoder;
//            case BackRight:
//                return backRightEncoder;
//            case FrontLeft:
//                return frontLeftEncoder;
//            case FrontRight:
//                return frontRightEncoder;
//            default:
//                throw new InvalidParameterException("Invalid servo motor position");
//        }
//    }
//
//    /*
//    Returns the servo based on the position value.
//     */
//    private CRServo getPositionServo(ServoPositionEnum servoPosition){
//        switch (servoPosition)
//        {
//            case BackLeft:
//                return backLeftServo;
//            case BackRight:
//                return backRightServo;
//            case FrontLeft:
//                return frontLeftServo;
//            case FrontRight:
//                return frontRightServo;
//            default:
//                throw new InvalidParameterException("Invalid servo motor position");
//        }
//    }
//
//    /*
//    Returns the pidController based on the position value.
//     */
//    private PidController getPositionPidController(ServoPositionEnum servoPosition){
//        switch (servoPosition)
//        {
//            case BackLeft:
//                return pidController1;
//            case BackRight:
//                return pidController2;
//            case FrontLeft:
//                return pidController3;
//            case FrontRight:
//                return pidController4;
//            default:
//                throw new InvalidParameterException("Invalid servo motor position");
//        }
//    }
}

