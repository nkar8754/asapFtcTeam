package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

// teleop start
@TeleOp
public class SwerveOpMode extends LinearOpMode {
    protected DcMotor frontLeftMotor;
    protected DcMotor frontRightMotor;
    protected DcMotor backLeftMotor;
    protected DcMotor backRightMotor;
    protected Servo frontLeftServo;
    protected Servo frontRightServo;
    protected Servo backLeftServo;
    protected Servo backRightServo;

    private PidController pidController = new PidController();

    @Override
    public void runOpMode() {
        this.initializeDcMotors();
        this.initializeServos();

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            this.executeRun();
        }
    }

    private void initializeDcMotors()
    {
        this.frontLeftMotor = hardwareMap.get(DcMotor.class, "frontLeftMotor");
        this.frontRightMotor = hardwareMap.get(DcMotor.class, "frontRightMotor");
        this.backLeftMotor = hardwareMap.get(DcMotor.class, "backLeftMotor");
        this.backRightMotor = hardwareMap.get(DcMotor.class, "backRightMotor");
    }

    private void initializeServos()
    {
        this.frontLeftServo = hardwareMap.get(Servo.class, "frontLeftServo");
        this.frontRightServo = hardwareMap.get(Servo.class, "frontRightServo");
        this.backLeftServo = hardwareMap.get(Servo.class, "backLeftServo");
        this.backRightServo = hardwareMap.get(Servo.class, "backRightServo");
    }

    private void setDcMotorTargetPower(double tgtPower)
    {
        this.frontLeftMotor.setPower(tgtPower);
        this.frontRightMotor.setPower(tgtPower);
        this.backLeftMotor.setPower(tgtPower);
        this.backRightMotor.setPower(tgtPower);
    }

    private void setServoPosition(double degrees)
    {
        this.frontRightServo.setPosition(degrees);
        this.frontLeftServo.setPosition(degrees);
        this.backLeftServo.setPosition(degrees);
        this.backRightServo.setPosition(degrees);
    }

    private void telemetryServoPosition()
    {
        telemetry.addData("Servo Position", frontLeftServo.getPosition());
        telemetry.addData("Servo Position", frontRightServo.getPosition());
        telemetry.addData("Servo Position", backLeftServo.getPosition());
        telemetry.addData("Servo Position", backRightServo.getPosition());
    }

    private void telemetryDcMotorPower()
    {
        telemetry.addData("Motor Power", frontLeftMotor.getPower());
        telemetry.addData("Motor Power", frontRightMotor.getPower());
        telemetry.addData("Motor Power", backLeftMotor.getPower());
        telemetry.addData("Motor Power", backRightMotor.getPower());
    }

    private void executeRun()
    {
        telemetry.addData("Status", "Running");
        telemetry.update();
        // run until the end of the match (driver presses STOP)
        double tgtPower = 0;
        tgtPower = -this.gamepad1.left_stick_y;
        this.setDcMotorTargetPower(tgtPower);

        telemetry.addData("Target Power", tgtPower);
        this.telemetryDcMotorPower();

        telemetry.addData("Status", "Running");
        telemetry.update();

        double currentPosition = frontLeftServo.getPosition();
        double position = 0;
        // check to see if we need to move the servo.
        if(gamepad1.y) {
            // move to 0 degrees.
            position = 0;
        } else if (gamepad1.b|| gamepad1.x) {
            // move to 90 degrees.
            position = 0.5;
        } else if (gamepad1.a) {
            // move to 180 degrees.
            position = 1;
        }
        double correctedPosition = this.pidController.getCorrectValue(position, currentPosition);
        this.setServoPosition(correctedPosition);

        this.telemetryServoPosition();
        telemetry.addData("Target Power", tgtPower);
        this.telemetryDcMotorPower();

        telemetry.addData("Status", "Running");
        telemetry.update();
    }
}