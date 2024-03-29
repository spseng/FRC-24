package frc.robot.drive;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
// import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.Constants;

import static frc.robot.Constants.*;

public final class SwerveMotor {

    // Instance Variables
    private final PIDController pidController = new PIDController(STEER_KP, STEER_KI, STEER_KD);
    private final double offset;
    private double dynamicOffset = 0;
    private final CANSparkMax steerMotor;
    private final CANSparkMax driveMotor;
    private final AbsoluteEncoder steerAbsoluteEncoder;

    private double prevAngle = 0;
    private double directionFactor = 1;

    private double latestDriveSpeed = 0;
    // private boolean directionInverted = false;

    private final DoublePublisher steeringReal;
    private final DoublePublisher steeringGoal;


    public SwerveMotor(int steerPort, int drivePort, double offset) {
        this.offset = offset;
        this.steerMotor = new CANSparkMax(steerPort, com.revrobotics.CANSparkLowLevel.MotorType.kBrushless);
        this.driveMotor = new CANSparkMax(drivePort, com.revrobotics.CANSparkLowLevel.MotorType.kBrushless);

        this.steerAbsoluteEncoder = this.steerMotor.getAbsoluteEncoder(com.revrobotics.SparkAbsoluteEncoder.Type.kDutyCycle);
    
        this.steeringReal = NetworkTableInstance.getDefault().getDoubleTopic("/steeringReal" + steerPort).publish();
        this.steeringGoal = NetworkTableInstance.getDefault().getDoubleTopic("/steeringGoal" + steerPort).publish();
    }

    public void calibrate() {
        this.steerMotor.getEncoder().setPosition(0);
        this.driveMotor.getEncoder().setPosition(0);
        // latestDriveSpeed = 0;
        // prevAngle = 0;

        this.dynamicOffset = getAbsoluteSteeringPosition();
    }

    public void zeroPosition() {
        steerMotor.set(pidController.calculate(getSteeringPosition(), this.getOffset()));
    }

    public void stopSteering() {
        steerMotor.set(0);
    }

    public void steer(double toAngle){
        double goalAngle = toAngle + prevAngle;
        steerMotor.set(pidController.calculate(prevAngle, goalAngle));

        steeringGoal.set(goalAngle);
        steeringReal.set(prevAngle);

        prevAngle = getSteeringPosition();
    }

    public void drive(double speed) {
        // driveMotor.setInverted(inverted);
        driveMotor.set(speed * directionFactor);

        latestDriveSpeed = speed * directionFactor;
    }
    public void setModuleState(SwerveModuleState state){
        double driveSpeed = state.speedMetersPerSecond * Constants.DRIVE_SPEED;
        double inDirection = state.angle.getRadians() / (2 * Math.PI) * Constants.FULL_ROTATION;

        double closestDirection = closestAngle(prevAngle, inDirection + this.getOffset()); //  % FULL_ROTATION + (directionFactor == -1 ? FULL_ROTATION/2 : 0)

        // if (Math.abs(closestDirection - (prevAngle)) % FULL_ROTATION > FULL_ROTATION/4) {
        //     directionFactor = -1;
        //     // driveSpeed *= -1;
        //     // closestDirection += FULL_ROTATION / 2;
        // }else {
        //     directionFactor = 1;
        // }

        // if(directionFactor == -1) {
        //     closestDirection += FULL_ROTATION/2;
        // }

        drive(driveSpeed);
        steer(closestDirection);
    }

    
    // Helper functions

    // This function is used to calculate the angle the wheel should be set to
    // based on the previous angle to determine which direction to turn

    // If the wheel is turning more than 90 degrees, then the wheel should spin in the opposite direction
    // and the drive wheel should spin in the opposite direction

    // https://compendium.readthedocs.io/en/latest/tasks/drivetrains/swerve.html
    private double closestAngle(double previous, double desiredAngle)
    {
        double goal = desiredAngle;
        // if ( directionFactor == -1) {
        //     goal += FULL_ROTATION/2;
        // }
        // get direction
        double dir = nearestRotation(goal) - nearestRotation(previous);
        
        // If rotation is greater than 180 degrees, then rotate swerve in the other way
        if (Math.abs(dir) > FULL_ROTATION/2)
        {
            dir = -(Math.signum(dir) * FULL_ROTATION) + dir;
        }

        return dir;
    }

    // For some reason the built-in modulo function didn't work...
    private static double nearestRotation(double angle)
    {
        return angle - FULL_ROTATION * Math.floor(angle / FULL_ROTATION);
    }


    // Getters and Setters
    public double getOffset() {
        return nearestRotation(offset + dynamicOffset);
    }
    public double getSteeringPosition() {
        return steerMotor.getEncoder().getPosition() / RELATIVE_ENCODER_CONVERSION * FULL_ROTATION;
    }
    public double getAbsoluteSteeringPosition() {
        return steerAbsoluteEncoder.getPosition() / ABS_ENCODER_CONVERSION * FULL_ROTATION;
    }

     public SwerveModulePosition getSwervePosition(){
         return new SwerveModulePosition(
            driveMotor.getEncoder().getPosition(), new Rotation2d((getSteeringPosition() + getOffset())/FULL_ROTATION * Math.PI * 2));
     }

    public SwerveModuleState getSwerveModuleState(){
        return new SwerveModuleState(latestDriveSpeed, new Rotation2d((prevAngle - getOffset())/FULL_ROTATION * Math.PI * 2));
    }
}