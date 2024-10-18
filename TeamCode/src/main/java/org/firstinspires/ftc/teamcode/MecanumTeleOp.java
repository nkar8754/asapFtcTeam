package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

    @TeleOp
    public class MecanumTeleOp extends LinearOpMode {
        private DcMotor frontLeftMotor;
        private DcMotor frontRightMotor;
        private DcMotor backLeftMotor;
        private DcMotor backRightMotor;
        private Servo frontLeftServo;
        private Servo frontRightServo;
        private Servo backLeftServo;
        private Servo backRightServo;

        @Override
        public void runOpMode() {

            frontLeftMotor = hardwareMap.get(DcMotor.class, "frontLeftMotor");
            frontRightMotor = hardwareMap.get(DcMotor.class, "frontRightMotor");
            backLeftMotor = hardwareMap.get(DcMotor.class, "backLeftMotor");
            backRightMotor = hardwareMap.get(DcMotor.class, "backRightMotor");

            frontLeftServo = hardwareMap.get(Servo.class, "frontLeftServo");
            frontRightServo = hardwareMap.get(Servo.class, "frontRightServo");
            backLeftServo = hardwareMap.get(Servo.class, "backLeftServo");
            backRightServo = hardwareMap.get(Servo.class, "backRightServo");

            telemetry.addData("Status", "Initialized");
            telemetry.update();
            // Wait for the game to start (driver presses PLAY)
            waitForStart();

            // run until the end of the match (driver presses STOP)
            while (opModeIsActive()) {
                telemetry.addData("Status", "Running");
                telemetry.update();
                // run until the end of the match (driver presses STOP)
                double tgtPower = 0;
                while (opModeIsActive()) {
                    tgtPower = -this.gamepad1.left_stick_y;
                    frontLeftMotor.setPower(tgtPower);
                    frontRightMotor.setPower(tgtPower);
                    backLeftMotor.setPower(tgtPower);
                    backRightMotor.setPower(tgtPower);

                    telemetry.addData("Target Power", tgtPower);
                    telemetry.addData("Motor Power", frontLeftMotor.getPower());
                    telemetry.addData("Motor Power", frontRightMotor.getPower());
                    telemetry.addData("Motor Power", backLeftMotor.getPower());
                    telemetry.addData("Motor Power", backRightMotor.getPower());

                    telemetry.addData("Status", "Running");
                    telemetry.update();
                    // run until the end of the match (driver presses STOP)
                    tgtPower = 0;
                    while (opModeIsActive()) {
                        tgtPower = -this.gamepad1.left_stick_y;
                        frontLeftMotor.setPower(tgtPower);
                        frontRightMotor.setPower(tgtPower);
                        backLeftMotor.setPower(tgtPower);
                        backRightMotor.setPower(tgtPower);

                        // check to see if we need to move the servo.
                        if(gamepad1.y) {
                            // move to 0 degrees.
                            frontRightServo.setPosition(0);
                            frontLeftServo.setPosition(0);
                            backLeftServo.setPosition(0);
                            backRightServo.setPosition(0);
                        } else if (gamepad1.b|| gamepad1.x) {
                            // move to 90 degrees.
                            frontRightServo.setPosition(0.5);
                            frontLeftServo.setPosition(0.5);
                            backLeftServo.setPosition(0.5);
                            backRightServo.setPosition(0.5);
                        } else if (gamepad1.a) {
                            // move to 180 degrees.
                            frontRightServo.setPosition(1);
                            frontLeftServo.setPosition(1);
                            backLeftServo.setPosition(1);
                            backRightServo.setPosition(1);
                        }

                        telemetry.addData("Servo Position", frontLeftServo.getPosition());
                        telemetry.addData("Servo Position", frontRightServo.getPosition());
                        telemetry.addData("Servo Position", backLeftServo.getPosition());
                        telemetry.addData("Servo Position", backRightServo.getPosition());
                        telemetry.addData("Target Power", tgtPower);
                        telemetry.addData("Motor Power", frontLeftMotor.getPower());
                        telemetry.addData("Motor Power", frontRightMotor.getPower());
                        telemetry.addData("Motor Power", backLeftMotor.getPower());
                        telemetry.addData("Motor Power", backRightMotor.getPower());

                        telemetry.addData("Status", "Running");
                        telemetry.update();
                    }
                }
            }
        }

    }
