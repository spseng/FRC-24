package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.math.controller.PIDController;

import static frc.robot.Constants.*;


public final class SwerveMotor {

    // Instance Variables
    private final PIDController pidController = new PIDController(KP, KI, KD);
    private final double offset;
    private final CANSparkMax steerMotor;
    private final  CANSparkMax driveMotor;

    private double prevAngle=0;


    public SwerveMotor(int steerPort, int drivePort, double offset) {
        this.offset = offset;
        this.steerMotor = new CANSparkMax(steerPort,MotorType.kBrushless);
        this.driveMotor = new CANSparkMax(drivePort,MotorType.kBrushless);
    }

    public void zeroPosition() {
        steerMotor.set(pidController.calculate(this.steerMotor.getEncoder().getPosition(), offset));
    }

    public void steer(double goalAngle){
        steerMotor.set(pidController.calculate(steerMotor.getEncoder().getPosition(), closestAngle(prevAngle, goalAngle)));

        prevAngle = goalAngle;
    }

    public void drive(double speed) {
        driveMotor.set(speed);
    }

    // Helper functions
    private double closestAngle(double a, double b){
        double angle = (b%2) - (a%2);

        // If past 180 degrees, go the other way
        if( Math.abs(angle)>1){
            angle = -Math.signum(angle)*2 + angle;
        }

        return angle;
    }
}