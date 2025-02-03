package org.firstinspires.ftc.teamcode;
import static org.firstinspires.ftc.teamcode.MathFunctions.AngleWrap;

import com.qualcomm.robotcore.util.Range;

public class RobotMovement {
    double worldXPosition = 0;
    double worldYPosition = 0;
    double worldAngle_rad = (-180);

    public void goToPosition(double x, double y, double movementSpeed, double preferredAngle, double turnSpeed){

        double distanceToTarget = Math.hypot(x - worldXPosition, y - worldYPosition);

        double absoluteAngleToTarget = Math.atan2(y - worldYPosition, x - worldXPosition);

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
