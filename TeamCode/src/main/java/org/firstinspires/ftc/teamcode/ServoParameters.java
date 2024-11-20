package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;

public class ServoParameters {
    private PidController pidController;
    private CRServo servo;
    private AnalogInput encoder;
    private double previousEncoderInDeg = 0;
    private int revolutionCount = 0;
    private double offset;

    public ServoParameters(
            PidController pidController,
            CRServo servo,
            AnalogInput encoder,
            double previousEncoderInDeg,
            int revolutionCount,
            double offset){
        this.pidController = pidController;
        this.servo = servo;
        this.encoder = encoder;
        this.previousEncoderInDeg = previousEncoderInDeg;
        this.revolutionCount = revolutionCount;
        this.offset = offset;
    }

    public PidController getPidController() {
        return pidController;
    }

    public CRServo getServo() {
        return servo;
    }

    public AnalogInput getEncoder() {
        return encoder;
    }

    public double getEncoderInDeg(){
        return (encoder.getVoltage() / 3.3) * 360.0;
    }

    public double getPreviousEncoderInDeg() {
        return previousEncoderInDeg;
    }

    public int getRevolutionCount() {
        return revolutionCount;
    }

    public double getOffset() {
        return offset;
    }
}
