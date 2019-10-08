package frc.robot;

import edu.wpi.first.wpilibj.Joystick;

class SRF_Swerve_Drive {
    Joystick xBox;
    double wheelBase,trackWidth,R;
    
    public SRF_Swerve_Drive(Joystick j, double w, double t, double r) {
        xBox = j;
        wheelBase = w;
        trackWidth = t;
        R = r;
    }

    public double[] convertToFieldPosition(double Y, double X, double gyroAngle){
        double newY, newX, temp;

        temp = Y*Math.cos(gyroAngle) + X*Math.sin(gyroAngle);
        newX = Y*-1*Math.sin(gyroAngle) + X*Math.cos(gyroAngle);
        newY = temp;

        return new double[] {newY,newX};
    }

    public void calculate(boolean orientToField){
        double X, Y, W, TemporaryForGyro;
        X = xBox.getRawAxis(0);
        Y = xBox.getRawAxis(1);
        W = xBox.getRawAxis(4);
        
        if(orientToField)
            convertToFieldPosition(X,Y, TemporaryForGyro);

        
    }
}
