package frc.robot;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;

import static frc.robot.Constants.*;


public final class SwerveMotor {

    // Instance Variables
    private final PIDController pidController = new PIDController(STEER_KP, STEER_KI, STEER_KD);
    private final double offset;
    private final CANSparkMax steerMotor;
    private final CANSparkMax driveMotor;
    private final RelativeEncoder steerEncoder;

    private double prevAngle = 0;
    private double directionFactor = 1;


    public SwerveMotor(int steerPort, int drivePort, double offset) {
        this.offset = offset;
        this.steerMotor = new CANSparkMax(steerPort,MotorType.kBrushless);
        this.driveMotor = new CANSparkMax(drivePort,MotorType.kBrushless);
        this.steerEncoder = this.steerMotor.getEncoder();
    }

    public void zeroPosition() {
        steerMotor.set(pidController.calculate(getSteeringPosition(), offset));
    }

    public void stopSteering() {
        steerMotor.set(0);
    }

    public void steer(double goalRotation){
        double goalAngle = closestAngle(prevAngle, goalRotation);
        System.out.println(goalAngle);
        double delta=(-prevAngle+goalRotation);
        if (Math.abs(delta)>0.6){
            steerMotor.set(pidController.calculate(getSteeringPosition(),prevAngle+2.375/2+delta));

        }else{
        steerMotor.set(pidController.calculate(getSteeringPosition(), goalRotation));
        }
        // // find closest angle to goal + 180
        // double goalAngleFlipped = closestAngle(prevAngle, goalRotation + 180.0);

        // // if the closest angle to setpoint is shorter
        // if (Math.abs(goalAngle) <= Math.abs(goalAngleFlipped))
        // {
        //     // unflip the motor direction use the setpoint
        //     directionFactor = 1;

        //     steerMotor.set(pidController.calculate(getSteeringPosition(), goalAngle * directionFactor));
        // }
        // // if the closest angle to setpoint + 180 is shorter
        // else
        // {
        //     // flip the motor direction and use the setpoint + 180
        //     directionFactor = -1;
        //     steerMotor.set(pidController.calculate(getSteeringPosition(), goalAngleFlipped * directionFactor));
        // }
        
        
        
        prevAngle = getSteeringPosition();
    }

    public void drive(double speed) {
        driveMotor.set(speed);
    }

    
    // Helper functions

    // This function is used to calculate the angle the wheel should be set to
    // based on the previous angle to determine which direction to turn
    // https://compendium.readthedocs.io/en/latest/tasks/drivetrains/swerve.html

    private static double closestAngle(double previous, double goal)
    {
        // get direction
        double dir = modulo(goal, FULL_ROTATION) - modulo(previous, FULL_ROTATION);

        // convert from -360 to 360 to -180 to 180
        if (Math.abs(dir) > FULL_ROTATION/2)
        {
            dir = -(Math.signum(dir) * FULL_ROTATION) + dir;
        }

        return dir;
    }

    private static double modulo(double a, double b)
    {
        return a - b * Math.floor(a / b);
    }
   
    // Getters and Setters
    public double getSteeringPosition() {
        return steerEncoder.getPosition();
    }
}