package frc.robot;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

import static frc.robot.Constants.*;

import javax.swing.plaf.synth.SynthStyle;


public final class SwerveMotor {

    // Instance Variables
    private final PIDController pidController = new PIDController(STEER_KP, STEER_KI, STEER_KD);
    private final double offset;
    private double dynamicOffset = 0;
    private final CANSparkMax steerMotor;
    private final CANSparkMax driveMotor;
    // private final RelativeEncoder driveEncoder;
    // private final RelativeEncoder steerEncoder;
    private final AbsoluteEncoder steerAbsoluteEncoder;

    private double prevAngle = 0;
    private double directionFactor = 1;


    public SwerveMotor(int steerPort, int drivePort, double offset) {
        this.offset = offset;
        this.steerMotor = new CANSparkMax(steerPort,MotorType.kBrushless);
        this.driveMotor = new CANSparkMax(drivePort,MotorType.kBrushless);
        // this.steerEncoder = this.steerMotor.getEncoder();
        // this.driveEncoder = this.driveMotor.getEncoder();
        this.steerAbsoluteEncoder = this.steerMotor.getAbsoluteEncoder(Type.kDutyCycle);
    }

    public void calibrate() {
        // while (Math.abs(getAbsoluteSteeringPosition() - offset) > 0.1) {
        //     steer(0.1 * pidController.calculate(getAbsoluteSteeringPosition(), offset));
        // }
        // this.steerEncoder.setPosition(0);
        // stopSteering();
//            System.out.println("ABS offset: " + getAbsoluteSteeringPosition());
//            System.out.println("rel offset: " + getSteeringPosition());
//            this.steerMotor.getEncoder().setPosition((getAbsoluteSteeringPosition() * 2.375/2) + STEERING_CALIBRATION_OFFSET);
//            System.out.println("Setting offset to: " + getSteeringPosition());
        // zeroPosition();

        this.dynamicOffset = getAbsoluteSteeringPosition();
    }

    public void zeroPosition() {
        steerMotor.set(pidController.calculate(getSteeringPosition(), this.getOffset()));
    }

    public void stopSteering() {
        steerMotor.set(0);
    }

    public void steer(double goalRotation){
        double goalAngle = prevAngle + closestAngle(prevAngle, goalRotation + this.getOffset());

        steerMotor.set(pidController.calculate(prevAngle, goalAngle));
        prevAngle = getSteeringPosition();
    }

    public void drive(double speed) {
        driveMotor.set(speed * directionFactor);
    }

    
    // Helper functions

    // This function is used to calculate the angle the wheel should be set to
    // based on the previous angle to determine which direction to turn
    // https://compendium.readthedocs.io/en/latest/tasks/drivetrains/swerve.html
    private static double closestAngle(double previous, double goal)
    {
        // get direction
        double dir = modulo(goal, FULL_ROTATION) - modulo(previous, FULL_ROTATION);
        
        // goal mod 1 - prev mod 1
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
        return steerMotor.getEncoder().getPosition()/2.375*2;
    }
    public double getAbsoluteSteeringPosition() {
        return steerAbsoluteEncoder.getPosition();
    }
    public SwerveModulePosition getSwervePosition(){
        return new SwerveModulePosition(
                driveMotor.getEncoder().getPosition(), new Rotation2d(getSteeringPosition()));
    }

    public double getOffset() {
        return offset + dynamicOffset;
    }
}