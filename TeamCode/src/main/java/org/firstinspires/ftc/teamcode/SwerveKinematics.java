package org.firstinspires.ftc.teamcode;

import java.lang.reflect.Array;
import java.util.ArrayList;
import java.util.Arrays;

public class SwerveKinematics {
    public matrix2d kinematicsMatrix = new matrix2d(new ArrayList<Integer>(Arrays.asList(3, 8)));

    public SwerveKinematics(double wt, double wb) {
        double rx = wb / 2;
        double ry = wt / 2;

        kinematicsMatrix.components = new ArrayList<Double>(Arrays.asList(
                1.0, 0.0, rx,
                0.0, 1.0, ry,
                1.0, 0.0, -rx,
                0.0, 1.0, ry,
                1.0, 0.0, -rx,
                0.0, 1.0, -ry,
                1.0, 0.0, rx,
                0.0, 1.0, -ry
        ));
    }

    public ArrayList<Double> getVelocities(double vx, double vy, double wx) {
        matrix2d velocity = new matrix2d(new ArrayList<Integer>(Arrays.asList(1, 3)));
        velocity.components = new ArrayList<Double>(Arrays.asList(vx, vy, wx));
        matrix2d velocities = matrix2d.matrixMultiply(this.kinematicsMatrix, velocity);

        double fr = Math.atan2(velocities.components.get(1), velocities.components.get(0));
        double fl = Math.atan2(velocities.components.get(3), velocities.components.get(2));
        double rl = Math.atan2(velocities.components.get(5), velocities.components.get(4));
        double rr = Math.atan2(velocities.components.get(7), velocities.components.get(6));

        return new ArrayList<Double>(Arrays.asList(
                //rr
                fr != fr ? 0 : fr,
                Math.sqrt(Math.pow(velocities.components.get(1), 2.0) + Math.pow(velocities.components.get(0), 2.0)),

                //rl
                fl != fl ? 0 : fl,
                Math.sqrt(Math.pow(velocities.components.get(3), 2.0) + Math.pow(velocities.components.get(2), 2.0)),

                //fl
                rl != rl ? 0 : rl,
                Math.sqrt(Math.pow(velocities.components.get(5), 2.0) + Math.pow(velocities.components.get(4), 2.0)),

                //fr
                rr != rr ? 0 : rr,
                Math.sqrt(Math.pow(velocities.components.get(7), 2.0) + Math.pow(velocities.components.get(6), 2.0))
        ));
    }
}