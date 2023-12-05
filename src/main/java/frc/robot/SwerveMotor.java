package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.math.controller.PIDController;

import static frc.robot.Constants.*;


public final class SwerveMotor {

    // Instance Variables
    private final PIDController pidController = new PIDController(STEER_KP, STEER_KI, STEER_KD);
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

    public void stopSteering() {
        steerMotor.set(0);
    }

    public void steer(double goalRotation){
        double closestAngle = closestAngle(prevAngle, goalRotation) + this.offset;

        steerMotor.set(pidController.calculate(steerMotor.getEncoder().getPosition(), closestAngle));

        prevAngle = steerMotor.getEncoder().getPosition();
    }

    public void drive(double speed) {
        driveMotor.set(speed);
    }

    
    // Helper functions

    // This function is used to calculate the angle the wheel should be set to
    // uses previous angle to determine which direction to turn
    // https://compendium.readthedocs.io/en/latest/tasks/drivetrains/swerve.html
    private static double closestAngle(double a, double b)
    {
        // get direction
        double dir = modulo(b, 2) - modulo(a, 2);

        // convert from -360 to 360 to -180 to 180
        if (Math.abs(dir) > 1)
        {
            dir = -(Math.signum(dir) * 2) + dir;
        }
        return dir;
    }

    private static double modulo(double a, double b)
    {
        return a - b * Math.floor(a / b);
    }
   
    // Getters and Setters
    public double getSteeringPosition() {
        return steerMotor.getEncoder().getPosition();
    }

}