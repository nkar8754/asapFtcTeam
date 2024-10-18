package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

    @TeleOp
    public class MecanumTeleOp extends LinearOpMode {
        private DcMotor motorTest;
        private Servo servoTest;

        @Override
        public void runOpMode() {
            motorTest = hardwareMap.get(DcMotor.class, "motorTest");
            servoTest = hardwareMap.get(Servo.class, "servoTest");

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
                    motorTest.setPower(tgtPower);
                    telemetry.addData("Target Power", tgtPower);
                    telemetry.addData("Motor Power", motorTest.getPower());
                    telemetry.addData("Status", "Running");
                    telemetry.update();
                    // run until the end of the match (driver presses STOP)
                    tgtPower = 0;
                    while (opModeIsActive()) {
                        tgtPower = -this.gamepad1.left_stick_y;
                        motorTest.setPower(tgtPower);
                        // check to see if we need to move the servo.
                        if(gamepad1.y) {
                            // move to 0 degrees.
                            servoTest.setPosition(0);
                        } else if (gamepad1.x || gamepad1.b) {
                            // move to 90 degrees.
                            servoTest.setPosition(0.5);
                        } else if (gamepad1.a) {
                            // move to 180 degrees.
                            servoTest.setPosition(1);
                        }

                        telemetry.addData("Servo Position", servoTest.getPosition());
                        telemetry.addData("Target Power", tgtPower);
                        telemetry.addData("Motor Power", motorTest.getPower());
                        telemetry.addData("Status", "Running");
                        telemetry.update();
                    }
                }
            }
        }

    }
