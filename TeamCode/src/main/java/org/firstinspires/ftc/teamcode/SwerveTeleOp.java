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

    static ArrayList<AnalyzedStone> internalStoneList = new ArrayList<>();
    static volatile ArrayList<AnalyzedStone> clientStoneList = new ArrayList<>();

    private DcMotor slide1;
    private DcMotor slide2;
    private Servo inclination;
    private Servo wrist;
    private Servo claw;

    private int slidePos = 0;
    private double clawAngle = -0.37;
    private double inclinationAngle = 0;
    private double wristAngle = 0.96;

    private DcMotor frontLeftMotor;
    private DcMotor backLeftMotor;
    private DcMotor frontRightMotor;
    private DcMotor backRightMotor;

    private CRServo frontLeftServo;
    private CRServo backLeftServo;
    private CRServo frontRightServo;
    private CRServo backRightServo;

    public static double kp = 2;
    public static double ki = 0.5;
    public static double kd = 0.0;

    public static double offsetFR = -40;
    public static double offsetBR = -60;
    public static double offsetFL = 100;
    public static double offsetBL = -150;

    AnalogInput backLeftEncoder;
    AnalogInput backRightEncoder;
    AnalogInput frontLeftEncoder;
    AnalogInput frontRightEncoder;

    private PidController pidController1;
    private PidController pidController2;
    private PidController pidController3;
    private PidController pidController4;

    private SwerveKinematics swerveController = new SwerveKinematics(234, 304.812);

    @Override
    public void runOpMode() {
        slide1 = hardwareMap.get(DcMotorEx.class, "slide1");
        slide2 = hardwareMap.get(DcMotorEx.class, "slide2");
        slide1.setTargetPosition(slidePos);
        slide2.setTargetPosition(slidePos);
        slide1.setPower(1);
        slide2.setPower(1);
        slide2.setDirection(DcMotorSimple.Direction.REVERSE);
        slide1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slide2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        inclination = hardwareMap.get(Servo.class, "inclination");
        wrist = hardwareMap.get(Servo.class, "wrist");
        claw = hardwareMap.get(Servo.class, "claw");

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

        StoneOrientationAnalysisPipeline  pipeline = new StoneOrientationAnalysisPipeline();

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        camera.setPipeline(pipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                /*
                 * This will be called if the camera could not be opened
                 */
            }
        });

        odometry = hardwareMap.get(SparkFunOTOS.class, "sensor_otos");
        odometry.setLinearUnit(DistanceUnit.INCH);
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

            ArrayList<Double> output = swerveController.getVelocities(-gamepad1.left_stick_y / 1.5, gamepad1.left_stick_x / 1.5, -gamepad1.right_stick_x / 360);
            drive(output);

            if (gamepad2.dpad_up) {
                slidePos += 7;
            } else if (gamepad2.dpad_down) {
                slidePos -= 7;
            }

            if (gamepad2.a && inclinationAngle <= 1.0) {
                inclinationAngle += 0.05;
            } else if (gamepad2.y && inclinationAngle >= 0.37) {
                inclinationAngle -= 0.05;
            }

            if (gamepad2.x && wristAngle <= 1.0) {
                wristAngle += 0.01;
            } else if (gamepad2.b && wristAngle >= -1.0) {
                wristAngle -= 0.01;
            }

            if (gamepad2.right_bumper && clawAngle <= 1.0) {
                clawAngle += 0.05;
            } else if (gamepad2.left_bumper && clawAngle >= -0.37) {
                clawAngle -= 0.05;
            }

            inclination.setPosition(inclinationAngle);
            wrist.setPosition(wristAngle);
            claw.setPosition(clawAngle);

            slide1.setTargetPosition(slidePos);
            slide2.setTargetPosition(slidePos);

            telemetry.addData("slidePos: ", slidePos);
            telemetry.addData("inclination: ", inclinationAngle);
            telemetry.addData("wrist: ", wristAngle);
            telemetry.addData("claw: ", clawAngle);
            telemetry.update();
        }
    }

    public void drive(ArrayList<Double> output) {
        double pid_output1 = -pidController1.calculate((((output.get(2) / Math.PI) + 1) / 2 + offsetBL / 360) % 1, (backLeftEncoder.getVoltage() / 3.3));
        backLeftServo.setPower(pid_output1 * 2);

        double pid_output2 = -pidController2.calculate((((output.get(0) / Math.PI) + 1) / 2 + offsetBR / 360) % 1, (backRightEncoder.getVoltage() / 3.3));
        backRightServo.setPower(pid_output2 * 2);

        double pid_output3 = -pidController3.calculate((((output.get(4) / Math.PI) + 1) / 2 + offsetFL / 360) % 1, (frontLeftEncoder.getVoltage() / 3.3));
        frontLeftServo.setPower(pid_output3 * 2);

        double pid_output4 = -pidController4.calculate((((output.get(6) / Math.PI) + 1) / 2 + offsetFR / 360) % 1, (frontRightEncoder.getVoltage() / 3.3));
        frontRightServo.setPower(pid_output4 * 2);

        if (Math.abs(pid_output1) < 0.3 && Math.abs(pid_output2) < 0.3 && Math.abs(pid_output3) < 0.3 && Math.abs(pid_output4) < 0.3) {
            frontLeftMotor.setPower(output.get(5));
            backLeftMotor.setPower(output.get(3));
            frontRightMotor.setPower(output.get(7));
            backRightMotor.setPower(output.get(1));
        } else {
            frontLeftMotor.setPower(0);
            backLeftMotor.setPower(0);
            frontRightMotor.setPower(0);
            backRightMotor.setPower(0);
        }
    }

    static class StoneOrientationAnalysisPipeline extends OpenCvPipeline {
        Mat colorMat = new Mat();
        Mat thresholdMat = new Mat();
        Mat thresholdMat1 = new Mat();
        Mat morphedThreshold = new Mat();
        Mat contoursOnPlainImageMat = new Mat();

        static final double DENSITY_UPRIGHT_THRESHOLD = 0.03;

        Mat erodeElement = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(3, 3));
        Mat dilateElement = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(6, 6));

        static final Scalar TEAL = new Scalar(3, 148, 252);
        static final Scalar PURPLE = new Scalar(158, 52, 235);
        static final Scalar RED = new Scalar(255, 0, 0);
        static final Scalar GREEN = new Scalar(0, 255, 0);
        static final Scalar BLUE = new Scalar(0, 0, 255);

        static final Scalar redLower = new Scalar(0, 70, 50);
        static final Scalar redUpper = new Scalar(10, 255, 255);
        static final Scalar redLower1 = new Scalar(170, 70, 50);
        static final Scalar redUpper1 = new Scalar(180, 255, 255);

        static final Scalar greenLower = new Scalar(20, 200, 50);
        static final Scalar greenUpper = new Scalar(102, 255, 255);

        static final Scalar blueLower = new Scalar(80, 100, 100);
        static final Scalar blueUpper = new Scalar(255, 255, 255);

        static final int CONTOUR_LINE_THICKNESS = 2;

        /*
         * Some stuff to handle returning our various buffers
         */
        enum Stage {
            FINAL,
            Cb,
            MASK,
            MASK_NR,
            CONTOURS;
        }

        Stage[] stages = Stage.values();

        // Keep track of what stage the viewport is showing
        int stageNum = 0;

        @Override
        public void onViewportTapped() {
            /*
             * Note that this method is invoked from the UI thread
             * so whatever we do here, we must do quickly.
             */

            int nextStageNum = stageNum + 1;

            if(nextStageNum >= stages.length) {
                nextStageNum = 0;
            }

            stageNum = nextStageNum;
        }

        @Override
        public Mat processFrame(Mat input) {
            // We'll be updating this with new data below
            internalStoneList.clear();

            /*
             * Run the image processing
             */
            for(MatOfPoint contour : findContours(input)) {
                analyzeContour(contour, input);
            }

            clientStoneList = new ArrayList<>(internalStoneList);

            /*
             * Decide which buffer to send to the viewport
             */
            switch (stages[stageNum]) {
                case Cb: {
                    return colorMat;
                }

                case FINAL: {
                    return input;
                }

                case MASK: {
                    return thresholdMat;
                }

                case MASK_NR: {
                    return morphedThreshold;
                }

                case CONTOURS: {
                    return contoursOnPlainImageMat;
                }
            }

            return input;
        }

        public ArrayList<AnalyzedStone> getDetectedStones() {
            return clientStoneList;
        }

        ArrayList<MatOfPoint> findContours(Mat input) {
            // A list we'll be using to store the contours we find
            ArrayList<MatOfPoint> contoursList = new ArrayList<>();

            Imgproc.cvtColor(input, colorMat, Imgproc.COLOR_RGB2HSV);

            Scalar lower = null;
            Scalar upper = null;

            if (targetColor == 0) {
                lower = redLower;
                upper = redUpper;
            } else if (targetColor == 1) {
                lower = greenLower;
                upper = greenUpper;
            } else if (targetColor == 2) {
                lower = blueLower;
                upper = blueUpper;
            }

            Core.inRange(colorMat, lower, upper, thresholdMat);

            if (targetColor == 0) {
                Core.inRange(colorMat, redLower1, redUpper1, thresholdMat1);
                Core.bitwise_or(thresholdMat, thresholdMat1, thresholdMat);
            }

            morphMask(thresholdMat, morphedThreshold);
            Imgproc.findContours(morphedThreshold, contoursList, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_NONE);

            // We do draw the contours we find, but not to the main input buffer.
            input.copyTo(contoursOnPlainImageMat);
            Imgproc.drawContours(contoursOnPlainImageMat, contoursList, -1, BLUE, CONTOUR_LINE_THICKNESS, 8);

            return contoursList;
        }

        void morphMask(Mat input, Mat output) {
            /*
             * Apply some erosion and dilation for noise reduction
             */

            Imgproc.erode(input, output, erodeElement);
            Imgproc.erode(output, output, erodeElement);

            Imgproc.dilate(output, output, dilateElement);
            Imgproc.dilate(output, output, dilateElement);
        }

        void analyzeContour(MatOfPoint contour, Mat input) {
            // Transform the contour to a different format
            Point[] points = contour.toArray();
            MatOfPoint2f contour2f = new MatOfPoint2f(contour.toArray());

            // Do a rect fit to the contour, and draw it on the screen
            RotatedRect rotatedRectFitToContour = Imgproc.minAreaRect(contour2f);
            drawRotatedRect(rotatedRectFitToContour, input);

            // The angle OpenCV gives us can be ambiguous, so look at the shape of
            // the rectangle to fix that.
            double rotRectAngle = rotatedRectFitToContour.angle;
            if (rotatedRectFitToContour.size.width < rotatedRectFitToContour.size.height) {
                rotRectAngle += 90;
            }

            // Figure out the slope of a line which would run through the middle, lengthwise
            // (Slope as in m from 'Y = mx + b')
            double midlineSlope = Math.tan(Math.toRadians(rotRectAngle));

            // We're going to split the this contour into two regions: one region for the points
            // which fall above the midline, and one region for the points which fall below.
            // We'll need a place to store the points as we split them, so we make ArrayLists
            ArrayList<Point> aboveMidline = new ArrayList<>(points.length/2);
            ArrayList<Point> belowMidline = new ArrayList<>(points.length/2);

            // Ok, now actually split the contour into those two regions we discussed earlier!
            for(Point p : points) {
                if(rotatedRectFitToContour.center.y - p.y > midlineSlope * (rotatedRectFitToContour.center.x - p.x)) {
                    aboveMidline.add(p);
                } else {
                    belowMidline.add(p);
                }
            }

            // Now that we've split the contour into those two regions, we analyze each
            // region independently.
            ContourRegionAnalysis aboveMidlineMetrics = analyzeContourRegion(aboveMidline);
            ContourRegionAnalysis belowMidlineMetrics = analyzeContourRegion(belowMidline);

            if(aboveMidlineMetrics == null || belowMidlineMetrics == null) {
                return; // Get out of dodge
            }

            // We're going to draw line from the center of the bounding rect, to outside the bounding rect, in the
            // direction of the side of the stone with the nubs.
            Point displOfOrientationLinePoint2 = computeDisplacementForSecondPointOfStoneOrientationLine(rotatedRectFitToContour, rotRectAngle);

            // Draw that line we were just talking about
            Imgproc.line(
                    input, // Buffer we're drawing on
                    new Point( // First point of the line (center of bounding rect)
                            rotatedRectFitToContour.center.x,
                            rotatedRectFitToContour.center.y),
                    new Point(
                            rotatedRectFitToContour.center.x-displOfOrientationLinePoint2.x,
                            rotatedRectFitToContour.center.y-displOfOrientationLinePoint2.y),
                    PURPLE,
                    2);

                Imgproc.drawContours(input, aboveMidlineMetrics.listHolderOfMatOfPoint, -1, TEAL, 2, 8);

                double angle = -(rotRectAngle-90);
                drawTagText(rotatedRectFitToContour, Integer.toString((int) Math.round(angle))+" deg", input);

                AnalyzedStone analyzedStone = new AnalyzedStone();
                analyzedStone.angle = angle;
                analyzedStone.rect = rotatedRectFitToContour;
                internalStoneList.add(analyzedStone);
        }

        static class ContourRegionAnalysis {
            /*
             * This class holds the results of analyzeContourRegion()
             */

            double hullArea;
            double contourArea;
            double density;
            List<MatOfPoint> listHolderOfMatOfPoint;
        }

        static ContourRegionAnalysis analyzeContourRegion(ArrayList<Point> contourPoints) {
            // drawContours() requires a LIST of contours (there's no singular drawContour()
            // method), so we have to make a list, even though we're only going to use a single
            // position in it...
            MatOfPoint matOfPoint = new MatOfPoint();
            matOfPoint.fromList(contourPoints);
            List<MatOfPoint> listHolderOfMatOfPoint = Arrays.asList(matOfPoint);

            // Compute the convex hull of the contour
            MatOfInt hullMatOfInt = new MatOfInt();
            Imgproc.convexHull(matOfPoint, hullMatOfInt);

            // Was the convex hull calculation successful?
            if(hullMatOfInt.toArray().length > 0) {
                // The convex hull calculation tells us the INDEX of the points which
                // which were passed in eariler which form the convex hull. That's all
                // well and good, but now we need filter out that original list to find
                // the actual POINTS which form the convex hull
                Point[] hullPoints = new Point[hullMatOfInt.rows()];
                List<Integer> hullContourIdxList = hullMatOfInt.toList();

                for (int i = 0; i < hullContourIdxList.size(); i++) {
                    hullPoints[i] = contourPoints.get(hullContourIdxList.get(i));
                }

                ContourRegionAnalysis analysis = new ContourRegionAnalysis();
                analysis.listHolderOfMatOfPoint = listHolderOfMatOfPoint;

                // Compute the hull area
                analysis.hullArea = Imgproc.contourArea(new MatOfPoint(hullPoints));

                // Compute the original contour area
                analysis.contourArea = Imgproc.contourArea(listHolderOfMatOfPoint.get(0));

                // Compute the contour density. This is the ratio of the contour area to the
                // area of the convex hull formed by the contour
                analysis.density = analysis.contourArea / analysis.hullArea;

                return analysis;
            } else {
                return null;
            }
        }

        static Point computeDisplacementForSecondPointOfStoneOrientationLine(RotatedRect rect, double unambiguousAngle) {
            // Note: we return a point, but really it's not a point in space, we're
            // simply using it to hold X & Y displacement values from the middle point
            // of the bounding rect.
            Point point = new Point();

            // Figure out the length of the short side of the rect
            double shortSideLen = Math.min(rect.size.width, rect.size.height);

            // We draw a line that's 3/4 of the length of the short side of the rect
            double lineLength = shortSideLen * .75;

            // The line is to be drawn at 90 deg relative to the midline running through
            // the rect lengthwise
            point.x = (int) (lineLength * Math.cos(Math.toRadians(unambiguousAngle+90)));
            point.y = (int) (lineLength * Math.sin(Math.toRadians(unambiguousAngle+90)));

            return point;
        }

        static void drawTagText(RotatedRect rect, String text, Mat mat) {
            Imgproc.putText(
                    mat, // The buffer we're drawing on
                    text, // The text we're drawing
                    new Point( // The anchor point for the text
                            rect.center.x-50,  // x anchor point
                            rect.center.y+25), // y anchor point
                    Imgproc.FONT_HERSHEY_PLAIN, // Font
                    1, // Font size
                    TEAL, // Font color
                    1); // Font thickness
        }

        static void drawRotatedRect(RotatedRect rect, Mat drawOn) {
            /*
             * Draws a rotated rect by drawing each of the 4 lines individually
             */

            Point[] points = new Point[4];
            rect.points(points);

            for(int i = 0; i < 4; i++) {
                Imgproc.line(drawOn, points[i], points[(i+1)%4], RED, 2);
            }
        }
    }
}
