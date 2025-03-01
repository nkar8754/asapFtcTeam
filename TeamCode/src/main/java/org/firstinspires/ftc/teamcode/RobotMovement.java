package org.firstinspires.ftc.teamcode;
import static org.firstinspires.ftc.teamcode.MathFunctions.AngleWrap;
import static org.firstinspires.ftc.teamcode.MathFunctions.lineCircleIntersection;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import static org.firstinspires.ftc.teamcode.Odometry.myOtos;

import com.qualcomm.robotcore.util.Range;

import org.opencv.core.Point;

import java.util.ArrayList;

public class RobotMovement {
    public static void followCurve(ArrayList<CurvePoint> allPoints, double followAngle){
        CurvePoint followMe = getFollowPointPath(allPoints, new Point(worldXPosition.x, worldYPosition.y), allPoints.get(0).followDistance);
        goToPosition(followMe.x, followMe.y, followMe.moveSpeed, followAngle, followMe.turnSpeed);
    }

    public static CurvePoint getFollowPointPath(ArrayList<CurvePoint> pathPoints, Point robotLocation, double followRadius){
        CurvePoint followMe = new CurvePoint(pathPoints.get(0));

        for(int i = 0; i < pathPoints.size() - 1; i ++){
            CurvePoint startLine = pathPoints.get(i);
            CurvePoint endLine = pathPoints.get(i + 1);

            ArrayList<Point> intersections = lineCircleIntersection(robotLocation, followRadius, startLine.toPoint(),
                    endLine.toPoint());

            double closestAngle = 1000;

            for(Point thisIntersection : intersections){
                double angle = Math.atan2(thisIntersection.y - worldYPosition.y, thisIntersection.x - worldXPosition.x);
                double deltaAngle = Math.abs(MathFunctions.AngleWrap(angle - worldAngle_rad));

                if(deltaAngle < closestAngle){
                    closestAngle = deltaAngle;
                    followMe.setPoint(thisIntersection);
                }
            }
        }
        return followMe;
    }

    static SparkFunOTOS.Pose2D worldXPosition = myOtos.getPosition();
    static SparkFunOTOS.Pose2D worldYPosition = myOtos.getPosition();
    static double worldAngle_rad = (-180);

    public static void goToPosition(double x, double y, double movementSpeed, double preferredAngle, double turnSpeed){

        double distanceToTarget = Math.hypot(x - worldXPosition.x, y - worldYPosition.y);

        double absoluteAngleToTarget = Math.atan2(y - worldYPosition.y, x - worldXPosition.x);

        double relativeAngleToPoint = AngleWrap(absoluteAngleToTarget - (worldAngle_rad - Math.toRadians(90)));

        double relativeXToPoint = Math.cos(relativeAngleToPoint) * distanceToTarget;
        double relativeYToPoint = Math.sin(relativeAngleToPoint) * distanceToTarget;

        double movementXPower = relativeXToPoint / (Math.abs(relativeXToPoint) + Math.abs(relativeYToPoint));
        double movementYPower = relativeYToPoint / (Math.abs(relativeXToPoint) + Math.abs(relativeYToPoint));

        double movement_x = movementXPower * movementSpeed;
        double movement_y = movementYPower * movementSpeed;

        double relativeTurnAngle = relativeAngleToPoint - Math.toRadians(180) + preferredAngle;
        double movement_turn = Range.clip(relativeTurnAngle / Math.toRadians(30), -1, 1) * turnSpeed;

        if (distanceToTarget < 10) {
            movement_turn = 0;
        }

    }
}
