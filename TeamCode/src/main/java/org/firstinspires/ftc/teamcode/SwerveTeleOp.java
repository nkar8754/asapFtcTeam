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
    public static double kd = 0.0;

    public static double offsetFR = 55;
    public static double offsetBR = 120;
    public static double offsetFL = 60;
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

    private SwerveKinematics swerveController = new SwerveKinematics(0, 0, 0, 13.75, 13.75);

    //private SwerveKinematics SwerveMove = new SwerveKinematics(1,2,3,16,16);

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

            swerveController.setInputs(-gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x);
            swerveController.calculateKinematics();

            frontLeftMotor.setPower(swerveController.Wheel1Speed);
            backLeftMotor.setPower(swerveController.Wheel3Speed);
            frontRightMotor.setPower(swerveController.Wheel2Speed);
            backRightMotor.setPower(swerveController.Wheel4Speed);

            double pid_output1 = -pidController1.calculate(((swerveController.Wheel3Angle + 200) / 400 + offsetBL / 360) % 1, (backLeftEncoder.getVoltage() / 3.3));
            backLeftServo.setPower(pid_output1 * 2);

            double pid_output2 = -pidController2.calculate(((swerveController.Wheel4Angle + 200) / 400 + offsetBR / 360) % 1, (backRightEncoder.getVoltage() / 3.3));
            backRightServo.setPower(pid_output2 * 2);

            double pid_output3 = -pidController3.calculate(((swerveController.Wheel1Angle + 200) / 400 + offsetFL / 360) % 1, (frontLeftEncoder.getVoltage() / 3.3));
            frontLeftServo.setPower(pid_output3 * 2);

            double pid_output4 = -pidController4.calculate(((swerveController.Wheel2Angle + 200) / 400 + offsetFR / 360) % 1, (frontRightEncoder.getVoltage() / 3.3));
            frontRightServo.setPower(pid_output4 * 2);

            telemetry.addData("Servo Power", pid_output1);
            telemetry.addData("EncoderBR", backRightEncoder.getVoltage() / 3.3);
            telemetry.addData("EncoderBL", backLeftEncoder.getVoltage() / 3.3);
            telemetry.addData("EncoderFR", frontRightEncoder.getVoltage() / 3.3);
            telemetry.addData("EncoderFL", frontLeftEncoder.getVoltage() / 3.3);

            telemetry.addData("w1", swerveController.Wheel1Speed);
            telemetry.addData("w2", swerveController.Wheel2Speed);
            telemetry.addData("w3", swerveController.Wheel3Speed);
            telemetry.addData("w4", swerveController.Wheel4Speed);

            telemetry.addData("Status", "Running");
            telemetry.update();
        }
    }

}
