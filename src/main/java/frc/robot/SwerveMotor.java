package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.math.controller.PIDController;

import static frc.robot.Constants.*;


public final class SwerveMotor {

    // Instance Variables
    private final PIDController pidController = new PIDController(STEER_KP, STEER_KI, STEER_KD);
    private final double offset;
    private final double steerFactor;
    private final CANSparkMax steerMotor;
    private final  CANSparkMax driveMotor;

    private double prevAngle=0;


    public SwerveMotor(int steerPort, int drivePort, double offset, boolean steerReverse) {
        this.offset = offset;
        this.steerFactor = steerReverse ? -1 : 1;
        this.steerMotor = new CANSparkMax(steerPort,MotorType.kBrushless);
        this.driveMotor = new CANSparkMax(drivePort,MotorType.kBrushless);
    }

    public void zeroPosition() {
        steerMotor.set(pidController.calculate(this.steerMotor.getEncoder().getPosition(), offset));
    }

    public void stopSteering() {
        steerMotor.set(0);
    }

    public void steer(double goalRotation){
        steerMotor.set(pidController.calculate(steerMotor.getEncoder().getPosition(), closestAngle(prevAngle, goalRotation * steerFactor + this.offset)));

        prevAngle = steerMotor.getEncoder().getPosition();
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
   
   
    // Getters and Setters
    public double getSteeringPosition() {
        return steerMotor.getEncoder().getPosition();
    }

}