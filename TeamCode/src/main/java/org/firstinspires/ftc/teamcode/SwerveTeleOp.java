package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
//import com.arcrobotics.ftclib.kinematics.wpilibkinematics.ChassisSpeeds;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
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

    public enum ServoPositionEnum {
        BackLeft,
        BackRight,
        FrontLeft,
        FrontRight
    }

    private ServoController backLeftServoController;
    private ServoController backRightServoController;
    private ServoController frontLeftServoController;
    private ServoController frontRightServoController;

    //private SwerveKinematics SwerveMove = new SwerveKinematics(1,2,3,16,16);

    @Override
    public void runOpMode() {

        initializeMotors();
        initializeServoControllers();

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

            backLeftServoController.SetServoPower(svP, Prop1, Int1, Deriv1, true);
            backRightServoController.SetServoPower(svP, Prop2, Int2, Deriv2, false);
            frontLeftServoController.SetServoPower(svP, Prop3, Int3, Deriv3, false);
            frontRightServoController.SetServoPower(svP, Prop4, Int4, Deriv4, false);

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

    private void initializeServoControllers() {
        backLeftServoController = new ServoController(ServoPositionEnum.BackLeft, hardwareMap, telemetry);
        backRightServoController = new ServoController(ServoPositionEnum.BackRight, hardwareMap, telemetry);
        frontLeftServoController = new ServoController(ServoPositionEnum.FrontLeft, hardwareMap, telemetry);
        frontRightServoController = new ServoController(ServoPositionEnum.FrontRight, hardwareMap, telemetry);
    }
}

