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
    public static double kd = 0.04;

    public static double offsetFR = 100;
    public static double offsetBR = 130;
    public static double offsetFL = -100;
    public static double offsetBL = -180;

    public static double svP = 0;

    AnalogInput backLeftEncoder;
    AnalogInput backRightEncoder;

    AnalogInput frontLeftEncoder;

    AnalogInput frontRightEncoder;

    private PidController pidController1 = new PidController(kp, ki, kd);
    private PidController pidController2 = new PidController(kp, ki, kd);
    private PidController pidController3 = new PidController(kp, ki, kd);
    private PidController pidController4 = new PidController(kp, ki, kd);

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

        // run until the end of the match (driver presses STOP)
        double tgtPower = 0;
        double servoPower = 0;

        double totalRotationBL = 0;
        double totalRotationBR = 0;
        double totalRotationFL = 0;
        double totalRotationFR = 0;

        int revolutionCountBL = 0;
        int revolutionCountBR = 0;
        int revolutionCountFL = 0;
        int revolutionCountFR = 0;

        double previousEncoderBL = 0;
        double previousEncoderBR = 0;
        double previousEncoderFL = 0;
        double previousEncoderFR = 0;

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

            double encoderBL = ((backLeftEncoder.getVoltage() / 3.3) * 360.0);
            double encoderBR = ((backRightEncoder.getVoltage() / 3.3) * 360.0);
            double encoderFL = ((frontLeftEncoder.getVoltage() / 3.3) * 360.0);
            double encoderFR = ((frontRightEncoder.getVoltage() / 3.3) * 360.0);

            double deltaAngleBL = Math.abs(encoderBL) - Math.abs(previousEncoderBL);
            if (deltaAngleBL < -180 && encoderBL < 180) {
                revolutionCountBL += 1;
            } else if (deltaAngleBL > 180 && encoderBL > 180) {
                revolutionCountBL -= 1;
            }

            totalRotationBL = encoderBL + revolutionCountBL * 360 - offsetBL;

            double deltaAngleBR = Math.abs(encoderBR) - Math.abs(previousEncoderBR);
            if (deltaAngleBR < -180 && encoderBR < 180) {
                revolutionCountBR += 1;
            } else if (deltaAngleBR > 180 && encoderBR > 180) {
                revolutionCountBR -= 1;
            }

            totalRotationBR = encoderBR + revolutionCountBR * 360 - offsetBR;

            double deltaAngleFL = Math.abs(encoderFL) - Math.abs(previousEncoderFL);
            if (deltaAngleFL < -180 && encoderFL < 180) {
                revolutionCountFL += 1;
            } else if (deltaAngleFL > 180 && encoderFL > 180) {
                revolutionCountFL -= 1;
            }

            totalRotationFL = encoderFL + revolutionCountFL * 360 - offsetFL;

            double deltaAngleFR = Math.abs(encoderFR) - Math.abs(previousEncoderFR);
            if (deltaAngleFR < -180 && encoderFR < 180) {
                revolutionCountFR += 1;
            } else if (deltaAngleFR > 180 && encoderFR > 180) {
                revolutionCountFR -= 1;
            }

            totalRotationFR = encoderFR + revolutionCountFR * 360 - offsetFR;

            previousEncoderBL = encoderBL;
            previousEncoderBR = encoderBR;
            previousEncoderFL = encoderFL;
            previousEncoderFR = encoderFR;

            double pid_output1 = -pidController1.calculate(servoPower, (totalRotationBL / 360.0) * 2.0 - 1.0);
            backLeftServo.setPower(pid_output1 * 2);

            double pid_output2 = -pidController2.calculate(servoPower, (totalRotationBR / 360.0) * 2.0 - 1.0);
            backRightServo.setPower(pid_output2 * 2);

            double pid_output3 = -pidController3.calculate(servoPower, (totalRotationFL / 360.0) * 2.0 - 1.0);
            frontLeftServo.setPower(pid_output3 * 2);

            double pid_output4 = -pidController4.calculate(servoPower, (totalRotationFR / 360.0) * 2.0 - 1.0);
            frontRightServo.setPower(pid_output4 * 2);

            telemetry.addData("Servo Power", pid_output1);
            telemetry.addData("EncoderBR", totalRotationBR);
            telemetry.addData("EncoderBL", totalRotationBL);
            telemetry.addData("EncoderFR", totalRotationFR);
            telemetry.addData("EncoderFL", totalRotationFL);

            telemetry.addData("revolutionsBR", revolutionCountBR * 100);
            telemetry.addData("revolutionsBL", revolutionCountBL * 100);
            telemetry.addData("revolutionsFR", revolutionCountFR * 100);
            telemetry.addData("revolutionsFL", revolutionCountFL * 100);

            telemetry.addData("Motor Power", tgtPower);
            telemetry.addData("Status", "Running");
            telemetry.update();
        }
    }

}

