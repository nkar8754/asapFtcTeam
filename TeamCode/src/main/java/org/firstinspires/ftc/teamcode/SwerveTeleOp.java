package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
//import com.arcrobotics.ftclib.kinematics.wpilibkinematics.ChassisSpeeds;
import com.acmerobotics.dashboard.config.Config;
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
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;

import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfInt;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera2;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Scanner;

@Config
@TeleOp
public class SwerveTeleOp extends LinearOpMode {
    OpenCvCamera camera;
    SparkFunOTOS odometry;
    SparkFunOTOS.Pose2D odoPos;

    public static int targetColor = 0;

    static class AnalyzedStone {
        RotatedRect rect;
        double angle;
    }

//    static ArrayList<AnalyzedStone> internalStoneList = new ArrayList<>();
//    static volatile ArrayList<AnalyzedStone> clientStoneList = new ArrayList<>();
//    private DcMotor slide1;
//    private DcMotor slide2;
//    private CRServo extension;
//    private CRServo inclination;
//    private Servo wrist;
//    private Servo claw;

//    private int slidePos = 0;
//    private double clawAngle = -0.37;
//    private double inclinationAngle = 2;
//    private double previousInclination = 2;
//    private int inclinationWrap = 1;
//    private double targetExtension = 0.04;
//    private double previousExtension = 0.04;
//    private int extensionWrap = 0;
//    private double wristAngle = 0.83;
//    private double readExtension = 0;
//    boolean grabbing = false;

    private DcMotor frontLeftMotor;
    private DcMotor backLeftMotor;
    private DcMotor frontRightMotor;
    private DcMotor backRightMotor;

    private DcMotor shooterMotor;

    private CRServo frontLeftServo;
    private CRServo backLeftServo;
    private CRServo frontRightServo;
    private CRServo backRightServo;

    private Servo topFlap;
    private Servo bottomFlap;
    private CRServo intake;
    private Motor intakeMotor;



    public static double kp = 2;
    public static double ki = 0.0;
    public static double kd = 1;
    public static double lkp = 2.0;
    public static double lki = 0.0;
    public static double lkd = 0.0;

    public static double offsetFR = 70;
    public static double offsetBR = 40;
    public static double offsetFL = 230;
    public static double offsetBL = -110;

    AnalogInput backLeftEncoder;
    AnalogInput backRightEncoder;
    AnalogInput frontLeftEncoder;
    AnalogInput frontRightEncoder;
//    AnalogInput extensionEncoder;
//    AnalogInput inclinationEncoder;

    private PidController pidController1;
    private PidController pidController2;
    private PidController pidController3;
    private PidController pidController4;
//    private GeneralPid inclinationController;
//    private GeneralPid extensionController;
    //private GeneralPid lateralController;

    private SwerveKinematics swerveController = new SwerveKinematics(234, 304.812);

    private double angleDiff(double a, double b) {
        double v1x = Math.cos(a * 2 * Math.PI);
        double v1y = Math.sin(a * 2 * Math.PI);

        double v2x = Math.cos(b * 2 * Math.PI);
        double v2y = Math.sin(b * 2 * Math.PI);

        return v1x * v2x + v1y * v2y;
    }

    private double getLaunchAngle(double posX, double posY, double v) {
        double D = Math.sqrt(Math.pow(0 - posX, 2) + Math.pow (0 - posY, 2));
        double H = 1.143;
        double A = 9.81;

        double a = (A * Math.pow(D, 2)) / (2 * Math.pow(v, 2));
        double c = a + H;

        double tanTheta = (D + Math.sqrt(Math.pow(D, 2) - 4 * a * c) / (2 * a));

        return Math.atan(tanTheta);
    }

    @Override
    public void runOpMode() {
//        slide1 = hardwareMap.get(DcMotorEx.class, "slide1");
//        slide2 = hardwareMap.get(DcMotorEx.class, "slide2");
//        slide1.setTargetPosition(slidePos);
//        slide2.setTargetPosition(slidePos);
//        slide1.setPower(1);
//        slide2.setPower(1);
//        slide2.setDirection(DcMotorSimple.Direction.REVERSE);
//        slide1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        slide2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        slide1.setMode(DcMotor.RunMode.RUN_TO_POSITION);5
//        slide2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//        extension = hardwareMap.get(CRServo.class, "extension");
//        inclination = hardwareMap.get(CRServo.class, "inclination");
//        wrist = hardwareMap.get(Servo.class, "wrist");
//        claw = hardwareMap.get(Servo.class, "claw");

        shooterMotor = hardwareMap.get(DcMotor.class, "shooterMotor");
        intakeMotor = hardwareMap.get(DcMotorEx.class, "intakeMotor");
        bottomFlap = hardwareMap.get(Servo.class, "bottomFlap");
        topFlap = hardwareMap.get(Servo.class, "topFlap");
        intake = hardwareMap.get(CRServo.class, "intake");


        shooterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);



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
//        inclinationEncoder = hardwareMap.get(AnalogInput.class, "inclinationEncoder");
//        extensionEncoder = hardwareMap.get(AnalogInput.class, "extenderEncoder");

        telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry(),telemetry);
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        //StoneOrientationAnalysisPipeline  pipeline = new StoneOrientationAnalysisPipeline();

        //int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        //camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        //camera.setPipeline(pipeline);
//        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
//            @Override
//            public void onOpened() {
//                camera.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
//            }
//
//            @Override
//            public void onError(int errorCode) {
//                /*
//                 * This will be called if the camera could not be opened
//                 */
//            }
//        });

        odometry = hardwareMap.get(SparkFunOTOS.class, "sensor_otos");
        odometry.setLinearUnit(DistanceUnit.METER);
        odometry.setAngularUnit(AngleUnit.DEGREES);
        odometry.setLinearScalar(1.008);
        odometry.setAngularScalar(0.992);
        odometry.calibrateImu();
        odometry.resetTracking();
        SparkFunOTOS.Pose2D currentPosition = new SparkFunOTOS.Pose2D(0, 0, 0);
        odometry.setPosition(currentPosition);

        waitForStart();

        pidController1 = new PidController(kp, ki, kd);
        pidController2 = new PidController(kp, ki, kd);
        pidController3 = new PidController(kp, ki, kd);
        pidController4 = new PidController(kp, ki, kd);
//        inclinationController = new GeneralPid(2, 0, 0);
//        extensionController = new GeneralPid(lkp, lki, lkd);
        //lateralController = new GeneralPid(0.5, 0, 1);

        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);



        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        while (opModeIsActive()) {

            for (LynxModule hub : allHubs) {
                hub.clearBulkCache();
            }

            double intakePower = 0;

// RB = intake
            if (gamepad1.left_bumper) {
                intakePower = 1;
            }
// LT = reverse
            else if (gamepad1.left_trigger > 0.05) {
                intakePower = -1;
            }
            else {
                intakePower = 0;
            }

            intake.setPower(intakePower);
            intakeMotor.setPower(intakePower);

            if (gamepad1.dpad_down) {
                bottomFlap.setPosition(1.0);
            } else if (gamepad1.dpad_up) {
                topFlap.setPosition(1.0);
            } else {
                bottomFlap.setPosition(0.0);
                topFlap.setPosition(0.0);
            }

            telemetry.update();


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
//            extensionController.Kp = lkp;
//            extensionController.Ki = lki;
//            extensionController.Kd = lkd;

//            if (gamepad2.dpad_up) {
//                slidePos += 10;
//            } else if (gamepad2.dpad_down && slidePos >= -40) {
//                slidePos -= 10;
//            }
//
//            double currentInclination = inclinationEncoder.getVoltage() / 3.3;
//
//            if (previousInclination - currentInclination < -0.5) {
//                inclinationWrap--;
//            } else if (previousInclination - currentInclination > 0.5) {
//                inclinationWrap++;
//            }
//
//            double actualInclination = inclinationWrap + currentInclination;
//
//            if (gamepad2.b && gamepad2.left_trigger > 0) {
//                targetColor = 0;
//            } else if (gamepad2.y && gamepad2.left_trigger > 0) {
//                targetColor = 1;
//            } else if (gamepad2.x && gamepad2.left_trigger > 0) {
//                targetColor = 2;
//            } else if (gamepad2.y && inclinationAngle <= 2.27) {
//                inclinationAngle += 0.03;
//            } else if (gamepad2.a && inclinationAngle >= 1.06) {
//                inclinationAngle -= 0.03;
//            } else if (gamepad2.x && wristAngle <= 1.0) {
//                wristAngle += 0.05;
//            } else if (gamepad2.b && wristAngle >= -1.0) {
//                wristAngle -= 0.05;
//            }
//
//            previousInclination = currentInclination;

//            double currentExtension = extensionEncoder.getVoltage() / 3.3;
//
//            if (previousExtension - currentExtension < -0.5) {
//                extensionWrap--;
//            } else if (previousExtension - currentExtension > 0.5) {
//                extensionWrap++;
//            }

//            double actualExtension = extensionWrap + currentExtension;
//
//            if (gamepad2.dpad_left && targetExtension <= 0.04) {
//                targetExtension += 0.03;
//            } else if (gamepad2.dpad_right && targetExtension >= -1.48) {
//                targetExtension -= 0.03;
//            }
//
//            previousExtension = currentExtension;
//
//            if (gamepad2.right_bumper && clawAngle <= 1.0) {
//                clawAngle += 0.05;
//            } else if (gamepad2.left_bumper && clawAngle >= -0.37) {
//                clawAngle -= 0.05;
//            }
//
//            if (gamepad2.left_trigger > 0.3 && gamepad2.a) {
//                slidePos = 875;
//            } else if (gamepad2.left_trigger > 0.3 && gamepad2.b) {
//                slidePos = 945;
//            }

//            double linkagePower = 0;
//            if (gamepad2.right_trigger > 0.3) {
//                int smallestIndex = -1;
//                double shortestDistance = 999.0;
//                ArrayList<AnalyzedStone> objectList = (ArrayList)clientStoneList.clone();
//
//                for (int i = 0; i < objectList.size(); i++) {
//                    AnalyzedStone object = objectList.get(i);
//                    double distance = Math.sqrt(Math.pow(object.rect.center.x - 317.0 / 2.0, 2.0) + Math.pow(object.rect.center.y - 237.0 / 2.0, 2.0));
//
//                    if (distance < shortestDistance) {
//                        shortestDistance = distance;
//                        smallestIndex = i;
//                    }
//                }
//
//                if (smallestIndex != -1 && !objectList.isEmpty() && grabbing == false) {
//                    AnalyzedStone targetObject = objectList.get(smallestIndex);
//                    wristAngle = mod(((targetObject.angle + 180) / 180), 1.0);
//                    wrist.setPosition(wristAngle);
//                    inclinationAngle = 1.07;
//                    targetExtension = actualExtension;
//
//                    linkagePower = -extensionController.calculate(0, (targetObject.rect.center.x / 317.0) - 0.5);
//                    extension.setPower(linkagePower * 0.5);
//
//                    double inclinationPower = -inclinationController.calculate(inclinationAngle, actualInclination);
//                    inclination.setPower(inclinationPower);
//
//                    if (Math.abs((targetObject.rect.center.x / 317.0) - 0.5) < 0.3 && Math.abs(1.07 - actualInclination) < 0.1) {
//                        readExtension = actualExtension;
//                        grabbing = true;
//                    }
//
//                    ArrayList<Double> output = swerveController.getVelocities(0, 0, 0);
//                    drive(output, 0);
//                } else if (grabbing == true) {
//                    clawAngle = 0.93;
//                    claw.setPosition(clawAngle);
//
//                    linkagePower = -extensionController.calculate(readExtension + 0.975, actualExtension);
//                    extension.setPower(linkagePower);
//
//                    targetExtension = readExtension + 0.975;
//                } else {
//                    extension.setPower(0);
//                    inclination.setPower(0);
//                }
            //} else {
//                grabbing = false;
                double speedMult = 1;

                if (gamepad1.right_bumper ) {
                    speedMult = 0.6;
                }

                double rotationRadians = (odometry.getPosition().h * Math.PI) / 180;
                matrix2d referenceTransform = new matrix2d(new ArrayList<Integer>(Arrays.asList(2, 2)));
                referenceTransform.components = new ArrayList<Double>(Arrays.asList(
                        Math.cos(rotationRadians), -Math.sin(rotationRadians),
                        Math.sin(rotationRadians), Math.cos(rotationRadians)
                ));

                matrix2d velocityWorld = new matrix2d(new ArrayList<Integer>(Arrays.asList(1, 2)));
                velocityWorld.components = new ArrayList<Double>(Arrays.asList(-gamepad1.left_stick_y / 1.5, gamepad1.left_stick_x / 1.5));
                velocityWorld = matrix2d.matrixMultiply(referenceTransform, velocityWorld);

                ArrayList<Double> output = swerveController.getVelocities(velocityWorld.components.get(0), velocityWorld.components.get(1), -gamepad1.right_stick_x / 360);
                drive(output, speedMult);
                          

//                linkagePower = -extensionController.calculate(targetExtension, actualExtension);
//                extension.setPower(linkagePower);
//                double inclinationPower = -inclinationController.calculate(inclinationAngle, actualInclination);
//                inclination.setPower(inclinationPower);
//                wrist.setPosition(wristAngle);
//                claw.setPosition(clawAngle);
           // }

//            slide1.setTargetPosition(slidePos);
//            slide2.setTargetPosition(slidePos);

//            telemetry.addData("slidePos: ", slidePos);
//            telemetry.addData("claw: ", clawAngle);
//            telemetry.addData("wrist: ", wristAngle);
//            telemetry.addData("extension: ", actualExtension);
//            telemetry.addData("targetExtension: ", targetExtension);
//            telemetry.addData("extensionPower: ", linkagePower);
//            telemetry.addData("inclination: ", actualInclination);
//            telemetry.addData("targetInclination: ", inclinationAngle);
            telemetry.addData("fr: ", frontRightEncoder.getVoltage() / 3.3);
            telemetry.addData("br: ", backRightEncoder.getVoltage() / 3.3);
            telemetry.addData("fl: ", frontLeftEncoder.getVoltage() / 3.3);
            telemetry.addData("bl: ", backLeftEncoder.getVoltage() / 3.3);
//            telemetry.addData("slide position: ", slide1.getCurrentPosition());
            telemetry.addData("X: ", odometry.getPosition().x);
            telemetry.addData("Y: ", odometry.getPosition().y);
            telemetry.addData("rot: ", odometry.getPosition().h);
            telemetry.update();
            if (gamepad1.right_trigger > 0.2) {
                shooterMotor.setPower(1);
            } else {
                shooterMotor.setPower(0);
            }

        }
    }

    private void drive(ArrayList<Double> output, double speedMult) {
        double angleBL = (((output.get(2) / Math.PI) + 1) / 2 + offsetBL / 360) % 1;
        double encoderBL = backLeftEncoder.getVoltage() / 3.3;
        double angleBLOpposite = (angleBL + 0.5) % 1;
        boolean blReverse = angleDiff(encoderBL, angleBL) < 0;
        if (blReverse) angleBL = angleBLOpposite;

        double pid_output1 = -pidController1.calculate(angleBL, encoderBL);
        backLeftServo.setPower(pid_output1 * 2);

        double angleBR = (((output.get(0) / Math.PI) + 1) / 2 + offsetBR / 360) % 1;
        double encoderBR = backRightEncoder.getVoltage() / 3.3;
        double angleBROpposite = (angleBR + 0.5) % 1;
        boolean brReverse = angleDiff(encoderBR, angleBR) < 0;
        if (brReverse) angleBR = angleBROpposite;

        double pid_output2 = -pidController2.calculate(angleBR, encoderBR);
        backRightServo.setPower(pid_output2 * 2);

        double angleFL = (((output.get(4) / Math.PI) + 1) / 2 + offsetFL / 360) % 1;
        double encoderFL = frontLeftEncoder.getVoltage() / 3.3;
        double angleFLOpposite = (angleFL + 0.5) % 1;
        boolean flReverse =  angleDiff(encoderFL, angleFL) < 0;
        if (flReverse) angleFL = angleFLOpposite;

        double pid_output3 = -pidController3.calculate(angleFL, encoderFL);
        frontLeftServo.setPower(pid_output3 * 2);

        double angleFR = (((output.get(6) / Math.PI) + 1) / 2 + offsetFR / 360) % 1;
        double encoderFR = frontRightEncoder.getVoltage() / 3.3;
        double angleFROpposite = (angleFR + 0.5) % 1;
        boolean frReverse = angleDiff(encoderFR, angleFR) < 0;
        if (frReverse) angleFR = angleFROpposite;

        double pid_output4 = -pidController4.calculate(angleFR, encoderFR);
        frontRightServo.setPower(pid_output4 * 2);

        if (Math.abs(pid_output1) < 0.55 && Math.abs(pid_output2) < 0.55 && Math.abs(pid_output3) < 0.55 && Math.abs(pid_output4) < 0.55) {
            frontLeftMotor.setPower(output.get(5) * speedMult * (flReverse ? -1 : 1));
            backLeftMotor.setPower(output.get(3) * speedMult * (blReverse ? -1 : 1));
            frontRightMotor.setPower(output.get(7) * speedMult * (frReverse ? -1 : 1));
            backRightMotor.setPower(output.get(1) * speedMult * (brReverse ? -1 : 1));
        } else {
            frontLeftMotor.setPower(0);
            backLeftMotor.setPower(0);
            frontRightMotor.setPower(0);
            backRightMotor.setPower(0);
        }
    }

//    static class StoneOrientationAnalysisPipeline extends OpenCvPipeline {
//        Mat colorMat = new Mat();
//        Mat thresholdMat = new Mat();
//        Mat thresholdMat1 = new Mat();
//        Mat morphedThreshold = new Mat();
//        Mat contoursOnPlainImageMat = new Mat();
//
//        static final double DENSITY_UPRIGHT_THRESHOLD = 0.03;
//
//        Mat erodeElement = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(3, 3));
//        Mat dilateElement = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(6, 6));
//
//        static final Scalar TEAL = new Scalar(3, 148, 252);
//        static final Scalar PURPLE = new Scalar(158, 52, 235);
//        static final Scalar RED = new Scalar(255, 0, 0);
//        static final Scalar GREEN = new Scalar(0, 255, 0);
//        static final Scalar BLUE = new Scalar(0, 0, 255);
//
//        static final Scalar redLower = new Scalar(0, 70, 50);
//        static final Scalar redUpper = new Scalar(10, 255, 255);
//        static final Scalar redLower1 = new Scalar(170, 70, 50);
//        static final Scalar redUpper1 = new Scalar(180, 255, 255);
//
//        static final Scalar greenLower = new Scalar(20, 200, 50);
//        static final Scalar greenUpper = new Scalar(102, 255, 255);
//
//        static final Scalar blueLower = new Scalar(80, 100, 100);
//        static final Scalar blueUpper = new Scalar(255, 255, 255);
//
//        static final int CONTOUR_LINE_THICKNESS = 2;
//
//        /*
//         * Some stuff to handle returning our various buffers
//         */
//        enum Stage {
//            FINAL,
//            Cb,
//            MASK,
//            MASK_NR,
//            CONTOURS;
//        }
//
//        Stage[] stages = Stage.values();
//
//        // Keep track of what stage the viewport is showing
//        int stageNum = 0;
//
//        @Override
//        public void onViewportTapped() {
//            /*
//             * Note that this method is invoked from the UI thread
//             * so whatever we do here, we must do quickly.
//             */
//
//            int nextStageNum = stageNum + 1;
//
//            if(nextStageNum >= stages.length) {
//                nextStageNum = 0;
//            }
//
//            stageNum = nextStageNum;
//        }
//
//        @Override
//        public Mat processFrame(Mat input) {
//            // We'll be updating this with new data below
//            internalStoneList.clear();
//
//            /*
//             * Run the image processing
//             */
//            for(MatOfPoint contour : findContours(input)) {
//                analyzeContour(contour, input);
//            }
//
//            clientStoneList = new ArrayList<>(internalStoneList);
//
//            /*
//             * Decide which buffer to send to the viewport
//             */
//            switch (stages[stageNum]) {
//                case Cb: {
//                    return colorMat;
//                }
//
//                case FINAL: {
//                    return input;
//                }
//
//                case MASK: {
//                    return thresholdMat;
//                }
//
//                case MASK_NR: {
//                    return morphedThreshold;
//                }
//
//                case CONTOURS: {
//                    return contoursOnPlainImageMat;
//                }
//            }
//
//            return input;
//        }
//
//        public ArrayList<AnalyzedStone> getDetectedStones() {
//            return clientStoneList;
//        }
//
//        ArrayList<MatOfPoint> findContours(Mat input) {
//            // A list we'll be using to store the contours we find
//            ArrayList<MatOfPoint> contoursList = new ArrayList<>();
//
//            Imgproc.cvtColor(input, colorMat, Imgproc.COLOR_RGB2HSV);
//
//            Scalar lower = null;
//            Scalar upper = null;
//
//            if (targetColor == 0) {
//                lower = redLower;
//                upper = redUpper;
//            } else if (targetColor == 1) {
//                lower = greenLower;
//                upper = greenUpper;
//            } else if (targetColor == 2) {
//                lower = blueLower;
//                upper = blueUpper;
//            }
//
//            Core.inRange(colorMat, lower, upper, thresholdMat);
//
//            if (targetColor == 0) {
//                Core.inRange(colorMat, redLower1, redUpper1, thresholdMat1);
//                Core.bitwise_or(thresholdMat, thresholdMat1, thresholdMat);
//            }
//
//            morphMask(thresholdMat, morphedThreshold);
//            Imgproc.findContours(morphedThreshold, contoursList, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_NONE);
//
//            // We do draw the contours we find, but not to the main input buffer.
//            input.copyTo(contoursOnPlainImageMat);
//            Imgproc.drawContours(contoursOnPlainImageMat, contoursList, -1, BLUE, CONTOUR_LINE_THICKNESS, 8);
//
//            return contoursList;
//        }
//
//        void morphMask(Mat input, Mat output) {
//            /*
//             * Apply some erosion and dilation for noise reduction
//             */
//
//            Imgproc.erode(input, output, erodeElement);
//            Imgproc.erode(output, output, erodeElement);
//
//            Imgproc.dilate(output, output, dilateElement);
//            Imgproc.dilate(output, output, dilateElement);
//        }
//
//        void analyzeContour(MatOfPoint contour, Mat input) {
//            // Transform the contour to a different format
//            Point[] points = contour.toArray();
//            MatOfPoint2f contour2f = new MatOfPoint2f(contour.toArray());
//
//            // Do a rect fit to the contour, and draw it on the screen
//            RotatedRect rotatedRectFitToContour = Imgproc.minAreaRect(contour2f);
//            drawRotatedRect(rotatedRectFitToContour, input);
//
//            // The angle OpenCV gives us can be ambiguous, so look at the shape of
//            // the rectangle to fix that.
//            double rotRectAngle = rotatedRectFitToContour.angle;
//            if (rotatedRectFitToContour.size.width < rotatedRectFitToContour.size.height) {
//                rotRectAngle += 90;
//            }
//
//            // Figure out the slope of a line which would run through the middle, lengthwise
//            // (Slope as in m from 'Y = mx + b')
//            double midlineSlope = Math.tan(Math.toRadians(rotRectAngle));
//
//            // We're going to split the this contour into two regions: one region for the points
//            // which fall above the midline, and one region for the points which fall below.
//            // We'll need a place to store the points as we split them, so we make ArrayLists
//            ArrayList<Point> aboveMidline = new ArrayList<>(points.length/2);
//            ArrayList<Point> belowMidline = new ArrayList<>(points.length/2);
//
//            // Ok, now actually split the contour into those two regions we discussed earlier!
//            for(Point p : points) {
//                if(rotatedRectFitToContour.center.y - p.y > midlineSlope * (rotatedRectFitToContour.center.x - p.x)) {
//                    aboveMidline.add(p);
//                } else {
//                    belowMidline.add(p);
//                }
//            }
//
//            // Now that we've split the contour into those two regions, we analyze each
//            // region independently.
//            ContourRegionAnalysis aboveMidlineMetrics = analyzeContourRegion(aboveMidline);
//            ContourRegionAnalysis belowMidlineMetrics = analyzeContourRegion(belowMidline);
//
//            if(aboveMidlineMetrics == null || belowMidlineMetrics == null) {
//                return; // Get out of dodge
//            }
//
//            // We're going to draw line from the center of the bounding rect, to outside the bounding rect, in the
//            // direction of the side of the stone with the nubs.
//            Point displOfOrientationLinePoint2 = computeDisplacementForSecondPointOfStoneOrientationLine(rotatedRectFitToContour, rotRectAngle);
//
//            // Draw that line we were just talking about
//            Imgproc.line(
//                    input, // Buffer we're drawing on
//                    new Point( // First point of the line (center of bounding rect)
//                            rotatedRectFitToContour.center.x,
//                            rotatedRectFitToContour.center.y),
//                    new Point(
//                            rotatedRectFitToContour.center.x-displOfOrientationLinePoint2.x,
//                            rotatedRectFitToContour.center.y-displOfOrientationLinePoint2.y),
//                    PURPLE,
//                    2);
//
//                Imgproc.drawContours(input, aboveMidlineMetrics.listHolderOfMatOfPoint, -1, TEAL, 2, 8);
//
//                double angle = -(rotRectAngle-90);
//                drawTagText(rotatedRectFitToContour, Integer.toString((int) Math.round(angle))+" deg", input);
//
//                AnalyzedStone analyzedStone = new AnalyzedStone();
//                analyzedStone.angle = angle;
//                analyzedStone.rect = rotatedRectFitToContour;
//                internalStoneList.add(analyzedStone);
//        }
//
//        static class ContourRegionAnalysis {
//            /*
//             * This class holds the results of analyzeContourRegion()
//             */
//
//            double hullArea;
//            double contourArea;
//            double density;
//            List<MatOfPoint> listHolderOfMatOfPoint;
//        }
//
//        static ContourRegionAnalysis analyzeContourRegion(ArrayList<Point> contourPoints) {
//            // drawContours() requires a LIST of contours (there's no singular drawContour()
//            // method), so we have to make a list, even though we're only going to use a single
//            // position in it...
//            MatOfPoint matOfPoint = new MatOfPoint();
//            matOfPoint.fromList(contourPoints);
//            List<MatOfPoint> listHolderOfMatOfPoint = Arrays.asList(matOfPoint);
//
//            // Compute the convex hull of the contour
//            MatOfInt hullMatOfInt = new MatOfInt();
//            Imgproc.convexHull(matOfPoint, hullMatOfInt);
//
//            // Was the convex hull calculation successful?
//            if(hullMatOfInt.toArray().length > 0) {
//                // The convex hull calculation tells us the INDEX of the points which
//                // which were passed in eariler which form the convex hull. That's all
//                // well and good, but now we need filter out that original list to find
//                // the actual POINTS which form the convex hull
//                Point[] hullPoints = new Point[hullMatOfInt.rows()];
//                List<Integer> hullContourIdxList = hullMatOfInt.toList();
//
//                for (int i = 0; i < hullContourIdxList.size(); i++) {
//                    hullPoints[i] = contourPoints.get(hullContourIdxList.get(i));
//                }
//
//                ContourRegionAnalysis analysis = new ContourRegionAnalysis();
//                analysis.listHolderOfMatOfPoint = listHolderOfMatOfPoint;
//
//                // Compute the hull area
//                analysis.hullArea = Imgproc.contourArea(new MatOfPoint(hullPoints));
//
//                // Compute the original contour area
//                analysis.contourArea = Imgproc.contourArea(listHolderOfMatOfPoint.get(0));
//
//                // Compute the contour density. This is the ratio of the contour area to the
//                // area of the convex hull formed by the contour
//                analysis.density = analysis.contourArea / analysis.hullArea;
//
//                return analysis;
//            } else {
//                return null;
//            }
//        }
//
//        static Point computeDisplacementForSecondPointOfStoneOrientationLine(RotatedRect rect, double unambiguousAngle) {
//            // Note: we return a point, but really it's not a point in space, we're
//            // simply using it to hold X & Y displacement values from the middle point
//            // of the bounding rect.
//            Point point = new Point();
//
//            // Figure out the length of the short side of the rect
//            double shortSideLen = Math.min(rect.size.width, rect.size.height);
//
//            // We draw a line that's 3/4 of the length of the short side of the rect
//            double lineLength = shortSideLen * .75;
//
//            // The line is to be drawn at 90 deg relative to the midline running through
//            // the rect lengthwise
//            point.x = (int) (lineLength * Math.cos(Math.toRadians(unambiguousAngle+90)));
//            point.y = (int) (lineLength * Math.sin(Math.toRadians(unambiguousAngle+90)));
//
//            return point;
//        }
//
//        static void drawTagText(RotatedRect rect, String text, Mat mat) {
//            Imgproc.putText(
//                    mat, // The buffer we're drawing on
//                    text, // The text we're drawing
//                    new Point( // The anchor point for the text
//                            rect.center.x-50,  // x anchor point
//                            rect.center.y+25), // y anchor point
//                    Imgproc.FONT_HERSHEY_PLAIN, // Font
//                    1, // Font size
//                    TEAL, // Font color
//                    1); // Font thickness
//        }
//
//        static void drawRotatedRect(RotatedRect rect, Mat drawOn) {
//            /*
//             * Draws a rotated rect by drawing each of the 4 lines individually
//             */
//
//            Point[] points = new Point[4];
//            rect.points(points);
//
//            for(int i = 0; i < 4; i++) {
//                Imgproc.line(drawOn, points[i], points[(i+1)%4], RED, 2);
//            }
//        }
//    }
}
