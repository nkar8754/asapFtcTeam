
package org.firstinspires.ftc.teamcode;


public class SwerveKinematics {
    private double Velocityx;
    private  double VelocityY;
    private double Angle;
    private double A;
    private double B;
    private double C;
    private double D;
    private double L;
    private double W;
    private double Wheel1Speed;
    private double Wheel2Speed;
    private double Wheel3Speed;
    private  double Wheel4Speed;
    private double Wheel1Angle;
    private double Wheel2Angle;
    private double Wheel3Angle;
    private double Wheel4Angle;

    private double[] speedsAngles = {Wheel1Speed,Wheel2Speed,Wheel3Speed,Wheel4Speed,Wheel1Angle,Wheel2Angle,Wheel3Angle,Wheel4Angle};




    public SwerveKinematics(double Vx, double Vy, double ang, double Width, double Length) {
        this.Velocityx = Vx;
        this.VelocityY = Vy;
        this.Angle = ang;
        this.W = Width;
        this.L = Length;

        A = Velocityx-Angle*L/2;
        B = Velocityx-Angle*W/2;
        C = VelocityY-Angle*L/2;
        D = VelocityY-Angle*W/2;

        Wheel1Speed = Math.sqrt((B*B)+(C*C));
        Wheel2Speed = Math.sqrt((B*B)+(D*D));
        Wheel3Speed = Math.sqrt((A*A)+(D*D));
        Wheel4Speed = Math.sqrt((A*A)+(C*C));

        Wheel1Angle = Math.atan2(B,D)*180/3.1415;
        Wheel2Angle = Math.atan2(B,C)*180/3.1415;
        Wheel3Angle = Math.atan2(A,D)*180/3.1415;
        Wheel4Angle = Math.atan2(A,C)*180/3.1415;



    }
    public double[] speedsAngles(){
        return speedsAngles;
    }


}
