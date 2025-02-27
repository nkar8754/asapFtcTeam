package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.RobotMovement.followCurve;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import java.util.ArrayList;
@Autonomous
public class Auto {

    public void runOpMode() {

        ArrayList<CurvePoint> allPoints = new ArrayList<>();
        allPoints.add(new CurvePoint(0,0, 1.0, 1.0, 50, Math.toRadians(50), 1.0));
        followCurve(allPoints, Math.toRadians(90));


        allPoints.add(new CurvePoint(0,0, 1.0, 1.0, 50, Math.toRadians(50), 1.0));
        followCurve(allPoints, Math.toRadians(90));

        allPoints.add(new CurvePoint(0,0, 1.0, 1.0, 50, Math.toRadians(50), 1.0));
        followCurve(allPoints, Math.toRadians(90));

        allPoints.add(new CurvePoint(0,0, 1.0, 1.0, 50, Math.toRadians(50), 1.0));
        followCurve(allPoints, Math.toRadians(90));

        allPoints.add(new CurvePoint(0,0, 1.0, 1.0, 50, Math.toRadians(50), 1.0));
        followCurve(allPoints, Math.toRadians(90));



}
