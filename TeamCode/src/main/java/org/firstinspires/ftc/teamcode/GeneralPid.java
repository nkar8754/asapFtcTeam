
package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.util.ElapsedTime;


public class GeneralPid {

    private double integral = 0;
    public double Kp = 0;
    public double Ki = 0;
    public double Kd = 0;

    private ElapsedTime timer = new ElapsedTime();
    private double lastError = 0;


    public GeneralPid(double p, double i, double d)
    {
        this.integral = 0;
        this.Kp = p;
        this.Ki = i;
        this.Kd = d;

        this.timer = new ElapsedTime();
        this.lastError = 0;
    }

    public double calculate(double targetState, double currentState)
    {
        double error = targetState - currentState;
        integral += error * timer.seconds();
        double derivative = (error - lastError) / timer.seconds();
        lastError = error;

        timer.reset();

        return (error * Kp) + (derivative * Kd) + (integral * Ki);
    }
}

