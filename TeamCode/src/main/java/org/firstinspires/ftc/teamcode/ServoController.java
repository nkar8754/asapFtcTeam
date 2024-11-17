package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class ServoController {

    private static double _prop = 0;
    private static double _int = 0;
    private static double _deriv = 0;

    private AnalogInput _encoder;
    private PidController _pidController;
    private CRServo _servo;
    private SwerveTeleOp.ServoPositionEnum _servoPosition;
    private HardwareMap _hardwareMap;
    private Telemetry _telemetry;

    public ServoController(SwerveTeleOp.ServoPositionEnum servoPosition, HardwareMap hardwareMap, Telemetry telemetry){
        _servoPosition = servoPosition;
        _hardwareMap = hardwareMap;
        _telemetry = telemetry;

        _pidController = new PidController(_prop, _int, _deriv);
        _servo = _hardwareMap.get(CRServo.class, servoPosition.name() + "Servo");
        _encoder = _hardwareMap.get(AnalogInput.class, servoPosition.name() + "Encoder");
    }

    public void SetServoPower(double svP, double proportional, double integral,double derivative,boolean displayServoPowerTelemetry)
    {
        _pidController.Kp = proportional;
        _pidController.Ki = integral;
        _pidController.Kd = derivative;

        double voltageAsDegrees = _encoder.getVoltage() / 3.3;

        double pid_output = -_pidController.calculate(svP, voltageAsDegrees*2-1);
        _servo.setPower(pid_output);

        if (displayServoPowerTelemetry) {
            _telemetry.addData("Servo Power", pid_output);
        }
        String encoderDegCaption = String.format("Encoder%s", _servoPosition.name());
        _telemetry.addData(encoderDegCaption, voltageAsDegrees * 360);
    }
}
