package org.firstinspires.ftc.teamcode;
import static org.firstinspires.ftc.teamcode.MathFunctions.AngleWrap;

public class RobotMovement {
    double worldXPosition = 0;
    double worldYPosition = 0;
    double worldAngle_rad = (-180);

    public void goToPosition(double x, double y, double movementSpeed){

        double distanceToTarget = Math.hypot(x - worldXPosition, y - worldYPosition);

        double absoluteAngleToTarget = Math.atan2(y - worldYPosition, x - worldXPosition);

        double relativeAngleToPoint = AngleWrap(absoluteAngleToTarget - (worldAngle_rad - Math.toRadians(90)));

        double relativeXToPoint = Math.cos(relativeAngleToPoint) * distanceToTarget;
        double relativeYToPoint = Math.sin(relativeAngleToPoint) * distanceToTarget;

        double movementXPower = relativeXToPoint / (Math.abs(relativeXToPoint) + Math.abs(relativeYToPoint));
        double movementYPower = relativeYToPoint / (Math.abs(relativeXToPoint) + Math.abs(relativeYToPoint));

        double movement_x = movementXPower * movementSpeed;
        double movement_y = movementYPower * movementSpeed;

    }
}
