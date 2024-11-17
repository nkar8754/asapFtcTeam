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

    public static double Prop1 = 0;
    public static double Int1 = 0;
    public static double Deriv1 = 0;

    public static double Prop2 = 0;
    public static double Int2 = 0;
    public static double Deriv2 = 0;

    public static double Prop3 = 0;
    public static double Int3 = 0;
    public static double Deriv3 = 0;

    public static double Prop4 = 0;
    public static double Int4 = 0;
    public static double Deriv4 = 0;

    public static double svP = 0;

    AnalogInput backLeftEncoder;
    AnalogInput backRightEncoder;
    AnalogInput frontLeftEncoder;
    AnalogInput frontRightEncoder;

    private PidController pidController1 = new PidController(Prop1, Int1, Deriv1);
    private PidController pidController2 = new PidController(Prop2, Int2, Deriv2);
    private PidController pidController3 = new PidController(Prop3, Int3, Deriv3);
    private PidController pidController4 = new PidController(Prop4, Int4, Deriv4);

    private enum ServoPositionEnum {
        BackLeft,
        BackRight,
        FrontLeft,
        FrontRight
    }

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

        // run until the end of the match (driver presses STOP)
        double tgtPower = 0;
        //double servoPower = 0;

        while (opModeIsActive()) {
            pidController1.Kp = Prop1;
            pidController1.Ki = Int1;
            pidController1.Kd = Deriv1;
            pidController2.Kp = Prop2;
            pidController2.Ki = Int2;
            pidController2.Kd = Deriv2;
            pidController3.Kp = Prop3;
            pidController3.Ki = Int3;
            pidController3.Kd = Deriv3;
            pidController4.Kp = Prop4;
            pidController4.Ki = Int4;
            pidController4.Kd = Deriv4;

//            if (Math.abs(this.gamepad1.left_stick_y) > 0.1) {
//                tgtPower = -this.gamepad1.left_stick_y;
//            } else {
//                tgtPower = 0;
//            }
//
//            if (Math.abs(this.gamepad1.right_stick_y) > 0.1) {
//                servoPower = -this.gamepad1.right_stick_y;
//            } else {
//                servoPower = 0;
//            }

            /**frontLeftMotor.setPower(tgtPower);
             backLeftMotor.setPower(tgtPower);
             frontRightMotor.setPower(tgtPower);
             backRightMotor.setPower(tgtPower);**/

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
        PidController pidController = getPositionPidController(servoPosition);
        AnalogInput encoder = getPositionEncoder(servoPosition);
        CRServo servo = getPositionServo(servoPosition);

        double voltageInDegrees = encoder.getVoltage() / 3.3;

        double pid_output = -pidController.calculate(svP, voltageInDegrees*2-1);
        servo.setPower(pid_output);

        if (displayServoPowerTelemetry) {
            telemetry.addData("Servo Power", pid_output);
        }
        String encoderDegCaption = String.format("Encoder%s", servoPosition.name());
        telemetry.addData(encoderDegCaption, voltageInDegrees * 360);
    }

    /*
    Returns the encoder based on the position value.
     */
    private AnalogInput getPositionEncoder(ServoPositionEnum servoPosition){
        switch (servoPosition)
        {
            case BackLeft:
                return backLeftEncoder;
            case BackRight:
                return backRightEncoder;
            case FrontLeft:
                return frontLeftEncoder;
            case FrontRight:
                return frontRightEncoder;
            default:
                throw new InvalidParameterException("Invalid servo motor position");
        }
    }

    /*
    Returns the servo based on the position value.
     */
    private CRServo getPositionServo(ServoPositionEnum servoPosition){
        switch (servoPosition)
        {
            case BackLeft:
                return backLeftServo;
            case BackRight:
                return backRightServo;
            case FrontLeft:
                return frontLeftServo;
            case FrontRight:
                return frontRightServo;
            default:
                throw new InvalidParameterException("Invalid servo motor position");
        }
    }

    /*
    Returns the pidController based on the position value.
     */
    private PidController getPositionPidController(ServoPositionEnum servoPosition){
        switch (servoPosition)
        {
            case BackLeft:
                return pidController1;
            case BackRight:
                return pidController2;
            case FrontLeft:
                return pidController3;
            case FrontRight:
                return pidController4;
            default:
                throw new InvalidParameterException("Invalid servo motor position");
        }
    }
}

