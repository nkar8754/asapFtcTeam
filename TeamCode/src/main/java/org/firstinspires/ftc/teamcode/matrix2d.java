package org.firstinspires.ftc.teamcode;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;

public class matrix2d {
    public ArrayList<Double> components;
    public ArrayList<Integer> shape;
    public matrix2d(ArrayList<Integer> shape) {
        int length = shape.get(0) * shape.get(1);

        components = new ArrayList<Double>(Collections.nCopies(length, 0.0));
        this.shape = shape;
    }

    public static matrix2d matrixMultiply(matrix2d m1, matrix2d m2) {
        int r1 = m1.shape.get(1);
        int c1 = m1.shape.get(0);
        int c2 = m2.shape.get(0);
        matrix2d result = new matrix2d(new ArrayList<Integer>(Arrays.asList(c2, r1)));

        for (int i = 0; i < r1; i++) {
            for (int j = 0; j < c2; j++) {
                for (int k = 0; k < c1; k++) {
                    result.components.set(j + i * result.shape.get(0), result.components.get(j + i * result.shape.get(0)) + m1.components.get(k + i * m1.shape.get(0)) * m2.components.get(j + k * m2.shape.get(0)));
                }
            }
        }

        return result;
    }
}