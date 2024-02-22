package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.vision.FieldLayout;

import static frc.robot.Constants.*;

public class ShooterSystem {


    
    // CANVenom intakeMotor = new CANVenom()
    // private final Spark intakeMotor;
    // private final Spark shooterMotor;

    private final CANSparkMax intakeMotor;
    private final CANSparkMax loadingMotor;
    private final CANSparkMax shooterMotor;
    private final CANSparkMax angleAlignmentMotor;
    private final RelativeEncoder angleEncoder;

    private final PIDController anglePIDController = new PIDController(SHOOTING_ANGLE_KP, SHOOTING_ANGLE_KI, SHOOTING_ANGLE_KD);


    private double shootEndDelay = 2.0;
    private double shootDelay = 1.5;
    private double shootDelayCounter = 0;


    private boolean isShooting = false;
    private boolean finishedShooting = false; // TODO: Find better way to tell if auton shot is done
//    private boolean isCalibrating = false;
    private double goalRotation = 0.0;
    private double goalAngle = 0.0;

    private final DigitalInput isLoadedButton;

    private final DigitalInput isLowestAngleButton;
    private final DigitalInput isHighestAngleButton;

    public ShooterSystem(int intakeMotorCAN, int loadingMotorCAN, int shooterMotorCAN, int angleAlignmentMotorCAN, int isLoadedButtonChannel, int isLowestAngleButtonChannel, int isHighestAngleButtonChannel) {
        intakeMotor = new CANSparkMax(intakeMotorCAN, MotorType.kBrushless);
        loadingMotor = new CANSparkMax(loadingMotorCAN, MotorType.kBrushless);
        shooterMotor = new CANSparkMax(shooterMotorCAN, MotorType.kBrushless);
        angleAlignmentMotor = new CANSparkMax(angleAlignmentMotorCAN, MotorType.kBrushless);
        angleEncoder = angleAlignmentMotor.getEncoder();

        isLoadedButton = new DigitalInput(isLoadedButtonChannel);
        isLowestAngleButton = new DigitalInput(isLowestAngleButtonChannel);
        isHighestAngleButton = new DigitalInput(isHighestAngleButtonChannel);

    }

    public void updateShuffleboard() {
        SmartDashboard.putBoolean("IntakeLoaded", isLoaded());
        SmartDashboard.putBoolean("Is Lowest", isLowestAngle());
        SmartDashboard.putBoolean("Is Highest", isHighestAngle());

        SmartDashboard.putNumber("Angle", angleEncoder.getPosition());
        SmartDashboard.putNumber("Goal Angle", goalRotation);
    }

    public void calibrate(){
        if(isLowestAngle()){
            angleAlignmentMotor.stopMotor();
            angleEncoder.setPosition(0);
        }
        setRotation(0);
    }

    public void setAngle(double angle){
        setRotation(angle*SHOOTER_ANGLE_CONVERSION - SHOOTER_RESTING_ANGLE);
    }

    public void setRotation(double angle){
        goalRotation = Math.max(angle, 0);
    }

    public void intakeUnlessLoaded(){
        if(!isLoaded()){
            intakeMotor.set(0.3);
            loadingMotor.set(0.3);
            shooterMotor.set(-0.05);
        } else {
            intakeMotor.stopMotor();
            loadingMotor.stopMotor();
            shooterMotor.stopMotor();
        }
    }

    public void rejectCurrentIntake(){
        intakeMotor.set(-0.3);
        loadingMotor.set(-0.3);
        shooterMotor.set(-0.3);
    }

    public void shootMaxSpeed(){
        shooterMotor.set(1.0);
        isShooting = true;
        shootDelayCounter = 0.0;
    }

    public void shootAmp(){
        setAngle(AMP_SCORING_ANGLE);
        shooterMotor.set(1.0);
        isShooting = true;
        shootDelayCounter = 0.0;
    }

    // Calculate the distance and shoot with appropriate speed regardless of alignment
    public void shoot(Pose2d fromPose){
        double horizontalDistance = fromPose.getTranslation().getNorm();
        SmartDashboard.putNumber("Shot from", horizontalDistance);

        double speed = 1.0;

        // TODO: Replace this with a continuous function
        // TODO: Test these rough values
//        if(horizontalDistance < 1.0){
//            speed = 0.5;
//        } else if(horizontalDistance < 2.0){
//            speed = 0.6;
//        } else if(horizontalDistance < 3.0){
//            speed = 0.7;
//        } else if(horizontalDistance < 4.0){
//            speed = 0.8;
//        } else if(horizontalDistance < 5.0){
//            speed = 0.9;
//        } else {
//            speed = 1.0;
//        }


        double exitVelocity = speed * SHOOTER_EXIT_VELOCITY; // TODO: Calculate exit velocity using the speed variable
        double angle = getAngle(horizontalDistance, exitVelocity);

        setAngle(angle);
        shooterMotor.set(speed);
        isShooting = true;
    }

    private static double getAngle(double horizontalDistance, double exitVelocity) {
        double verticalDistance = GOAL_HEIGHT - ROBOT_SHOOTER_HEIGHT; // TODO: Make sure this value is accurate
        double totalDistance = Math.sqrt(Math.pow(horizontalDistance, 2) + Math.pow(verticalDistance, 2));

        // Calculate the angle using these values
        double estimatedTime = totalDistance / exitVelocity;
        double approximatedLinearGoal = verticalDistance + GRAVITY * Math.pow(estimatedTime, 2);

        // Shoot an angle pointed directly at the approximatedLinearGoal
        return Math.toDegrees(Math.atan(approximatedLinearGoal / horizontalDistance));
    }

    public boolean autonShoot(Pose2d pose) {
        if(!isShooting) {
            shoot(pose);
        } else return finishedShooting;

        return false;
    }

    public boolean autonShootAmp() {
        if(!isShooting) {
            shootAmp();
        } else {
            return finishedShooting;
        }

        return false;
    }

    public void periodic(double dt) {
        finishedShooting = false;

        if (isShooting && Math.abs(getAngle() - goalAngle) < SHOOTING_ANGLE_ERROR) {
            if (shootDelayCounter < shootDelay) {
                shootDelayCounter += dt;
                shooterMotor.set(1.0);
            } else if (shootDelayCounter < shootEndDelay) {
                shootDelayCounter += dt;
                loadingMotor.set(0.5);
            } else {
                intakeMotor.stopMotor();
                loadingMotor.stopMotor();
                shooterMotor.stopMotor();
                isShooting = false;
                finishedShooting = true;
            }
        }

        double angleMoveSpeed = anglePIDController.calculate(angleEncoder.getPosition(), goalRotation);
        angleAlignmentMotor.set(angleMoveSpeed);

        if(isLowestAngle()){
            angleAlignmentMotor.stopMotor();
            angleEncoder.setPosition(0);

            if (goalRotation < 0) {
                setRotation(0);
            }
        }

        if(isHighestAngle()){
            goalRotation = Math.min(goalRotation, angleEncoder.getPosition());
        }

        updateShuffleboard();
    }

    public double getAngle(){
        return angleEncoder.getPosition() / SHOOTER_ANGLE_CONVERSION + SHOOTER_RESTING_ANGLE;
    }

    public boolean isLoaded(){
        return !isLoadedButton.get();
    }

    public boolean isLowestAngle(){
        return !isLowestAngleButton.get();
    }

    public boolean isHighestAngle(){
        return !isHighestAngleButton.get();
    }
}
