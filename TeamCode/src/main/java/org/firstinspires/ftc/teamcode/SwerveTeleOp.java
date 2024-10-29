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

    public static double Prop = 0.1;
    public static double Int = 0;

    public static double Deriv = 2;

    public static double svP = 0;

    AnalogInput backLeftEncoder;
    AnalogInput backRightEncoder;

    AnalogInput frontLeftEncoder;

    AnalogInput frontRightEncoder;

    private PidController pidController = new PidController(Prop, Int, Deriv);

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
        //double servoPower = 0;

        while (opModeIsActive()) {
            pidController.Kp = Prop;
            pidController.Ki = Int;
            pidController.Kd = Deriv;

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
            double pid_output1 = -pidController.calculate(svP, (backLeftEncoder.getVoltage() / 3.3)*2-1);
            backLeftServo.setPower(pid_output1);
            double position1 = (backLeftEncoder.getVoltage() / 3.3) * 360;

//            double pid_output2 = pidController.calculate(svP, (backRightEncoder.getVoltage() / 3.3)*2-1);
//            backRightServo.setPower(pid_output2);
//            double position2 = (backRightEncoder.getVoltage() / 3.3) * 360;
//
//            double pid_output3 = pidController.calculate(svP, (frontLeftEncoder.getVoltage() / 3.3)*2-1);
//            frontLeftServo.setPower(pid_output3);
//            double position3 = (frontLeftEncoder.getVoltage() / 3.3) * 360;

//            double pid_output4 = pidController.calculate(svP, (backRightEncoder.getVoltage() / 3.3)*2-1);
//            frontRightServo.setPower(pid_output4);
//            double position4 = (frontRightEncoder.getVoltage() / 3.3) * 360;


            telemetry.addData("Servo Power", pid_output1);
            telemetry.addData("Encoder1", position1);
            telemetry.addData("Motor Power", tgtPower);
            telemetry.addData("Status", "Running");
            telemetry.update();
        }
    }

}

