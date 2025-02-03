package org.firstinspires.ftc.teamcode;

import org.opencv.core.Point;

import java.util.ArrayList;

public class MathFunctions {
    /**
    Makes sure an angle is within the range -180 to 180 degrees
    **/
    public static double AngleWrap(double angle){
        while(angle < -Math.PI){
            angle += 2 * Math.PI;
        }
        while(angle > Math.PI){
            angle -= 2 * Math.PI;
        }
        return angle;
    }

    public static ArrayList<Point> lineCircleIntersection(Point circleCenter, double radius, Point linePoint1, Point linePoint2){

        if(Math.abs(linePoint1.y - linePoint2.y) < 0.003){
            linePoint1.y = linePoint2.y + 0.003;
        }
        if(Math.abs(linePoint1.x - linePoint2.x) < 0.003){
            linePoint1.x = linePoint2.x + 0.003;
        }

        double m1 = (linePoint2.y - linePoint1.y) / (linePoint2.x - linePoint1.x);

        double quadraticA = 1.0 + Math.pow(m1,2);

        double x1 = linePoint1.x - circleCenter.x;
        double y1 = linePoint1.y - circleCenter.y;

    }
}
