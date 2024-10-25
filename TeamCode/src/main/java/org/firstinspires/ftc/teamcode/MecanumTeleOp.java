package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

// teleop start
    @TeleOp
    public class MecanumTeleOp extends LinearOpMode {
        public DcMotor frontLeftMotor;
        public DcMotor frontRightMotor;
        public DcMotor backLeftMotor;
        public DcMotor backRightMotor;
        public Servo frontLeftServo;
        public Servo frontRightServo;
        public Servo backLeftServo;
        public Servo backRightServo;

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
            frontLeftMotor = hardwareMap.get(DcMotor.class, "frontLeftMotor");
            frontRightMotor = hardwareMap.get(DcMotor.class, "frontRightMotor");
            backLeftMotor = hardwareMap.get(DcMotor.class, "backLeftMotor");
            backRightMotor = hardwareMap.get(DcMotor.class, "backRightMotor");
        }

        private void initializeServos()
        {
            frontLeftServo = hardwareMap.get(Servo.class, "frontLeftServo");
            frontRightServo = hardwareMap.get(Servo.class, "frontRightServo");
            backLeftServo = hardwareMap.get(Servo.class, "backLeftServo");
            backRightServo = hardwareMap.get(Servo.class, "backRightServo");
        }

        private void executeRun()
        {
            telemetry.addData("Status", "Running");
            telemetry.update();
            // run until the end of the match (driver presses STOP)
            double tgtPower = 0;
            while (opModeIsActive()) {
                tgtPower = -this.gamepad1.left_stick_y;
                this.setDcMotorTargetPower(tgtPower);

                telemetry.addData("Target Power", tgtPower);
                this.telemetryDcMotorPower();

                telemetry.addData("Status", "Running");
                telemetry.update();
                // run until the end of the match (driver presses STOP)
                tgtPower = 0;
                while (opModeIsActive()) {
                    tgtPower = -this.gamepad1.left_stick_y;
                    this.setDcMotorTargetPower(tgtPower);

                    // check to see if we need to move the servo.
                    if(gamepad1.y) {
                        // move to 0 degrees.
                        this.setServoPosition(0);
                    } else if (gamepad1.b|| gamepad1.x) {
                        // move to 90 degrees.
                        this.setServoPosition(0.5);
                    } else if (gamepad1.a) {
                        // move to 180 degrees.
                        this.setServoPosition(1);
                    }

                    this.telemetryServoPosition();
                    telemetry.addData("Target Power", tgtPower);
                    this.telemetryDcMotorPower();

                    telemetry.addData("Status", "Running");
                    telemetry.update();
                }
            }
        }

        private void setDcMotorTargetPower(double tgtPower)
        {
            frontLeftMotor.setPower(tgtPower);
            frontRightMotor.setPower(tgtPower);
            backLeftMotor.setPower(tgtPower);
            backRightMotor.setPower(tgtPower);
        }

        private void setServoPosition(double degrees)
        {
            frontRightServo.setPosition(degrees);
            frontLeftServo.setPosition(degrees);
            backLeftServo.setPosition(degrees);
            backRightServo.setPosition(degrees);
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
    }
