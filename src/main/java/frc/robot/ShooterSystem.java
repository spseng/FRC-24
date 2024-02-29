package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.*;


import static frc.robot.Constants.*;

public class ShooterSystem {

    // CANVenom intakeMotor = new CANVenom()
    // private final Spark intakeMotor;
    // private final Spark shooterMotor;

    private final CANSparkMax intakeMotor;
    private final CANSparkMax loadingMotor;
    private final CANSparkMax shooterMotor;
    // private final CANSparkMax angleAlignmentMotor;

//    private final RelativeEncoder angleEncoder;
    private final WPI_CANCoder angleEncoder;
    private final WPI_TalonFX angleAlignmentMotor;

    private final PIDController anglePIDController = new PIDController(SHOOTING_ANGLE_KP, SHOOTING_ANGLE_KI, SHOOTING_ANGLE_KD);


    private double shootEndDelay = 2.0;
    private double shootDelay = 1.5;
    private double shootDelayCounter = 0;


    private boolean isShooting = false;
    private boolean finishedShooting = false; // TODO: Find better way to tell if auton shot is done
//    private boolean isCalibrating = false;
    private double goalRotation = 0.0;

    private final DigitalInput isLoadedButton;
    private final DigitalInput isLowestAngleButton;
    private final DigitalInput isHighestAngleButton;

    public ShooterSystem(int intakeMotorCAN, int loadingMotorCAN, int shooterMotorCAN, int angleAlignmentMotorCAN, int angleAlignmentEncoderCAN, int isLoadedButtonChannel, int isLowestAngleButtonChannel, int isHighestAngleButtonChannel) {
        intakeMotor = new CANSparkMax(intakeMotorCAN, MotorType.kBrushless);
        intakeMotor.setInverted(true);

        loadingMotor = new CANSparkMax(loadingMotorCAN, MotorType.kBrushless);
        loadingMotor.setInverted(true);

        shooterMotor = new CANSparkMax(shooterMotorCAN, MotorType.kBrushless);
        // angleAlignmentMotor = new CANSparkMax(angleAlignmentMotorCAN, MotorType.kBrushless);
        angleEncoder = new WPI_CANCoder(angleAlignmentEncoderCAN);
        angleAlignmentMotor = new WPI_TalonFX(angleAlignmentMotorCAN);
        // angleEncoder = angleAlignmentMotor.getEncoder();

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

        SmartDashboard.putNumber("Shooter Counter", shootDelayCounter);
    }

    public void calibrate(){
        if(isLowestAngle()){
            angleAlignmentMotor.stopMotor();
            // angleEncoder.setPosition(AUTONOMOUS_POSITION_MAX_ERROR)(0);
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
            System.out.println("Spinning the things");
            intakeMotor.set(INTAKE_SPEED);
            loadingMotor.set(LOADING_SPEED);
            shooterMotor.set(-INTAKE_SPEED);
        } else {
            intakeMotor.stopMotor();
            loadingMotor.stopMotor();
            shooterMotor.stopMotor();
        }
    }

    public void rotateAngle(double amount) {
        if((amount > 0 && !isHighestAngle()) || (amount < 0 && !isLowestAngle())) {
            goalRotation += amount/SHOOTER_ANGLE_CONVERSION;
        }
    }

    public void rejectCurrentIntake(){
        intakeMotor.set(-INTAKE_SPEED);
        loadingMotor.set(-LOADING_SPEED);
        shooterMotor.set(-INTAKE_SPEED);
    }

    public void stopIntake() {
        if(!isShooting){
            intakeMotor.stopMotor();
            loadingMotor.stopMotor();
            shooterMotor.stopMotor();
        }
    }

    public void shootMaxSpeed(){
        shooterMotor.set(SHOOT_STATIC_SPEED);
        isShooting = true;
        shootDelayCounter = 0.0;
    }

    public void shootAmp(){
        setAngle(AMP_SCORING_ANGLE);
        shooterMotor.set(SHOOT_STATIC_SPEED);
        isShooting = true;
        shootDelayCounter = 0.0;
    }

    public void shoot(Pose2d fromPose){
        double calculatedAngle = calculateAngle(fromPose);
        setAngle(calculatedAngle);
        shooterMotor.set(calculateSpeed(calculatedAngle, fromPose));
        isShooting = true;
        shootDelayCounter = 0.0;
    }

    public void lineUpAngle(Pose2d fromPose){
        double angle = calculateAngle(fromPose);
        setAngle(angle);
    }

    public double calculateAngle(Pose2d fromPose){
        // Constants
        double g = -9.81; // Gravity (m/s^2)
        double h0 = ROBOT_SHOOTER_HEIGHT; // Shooter height (m)
        double hg = GOAL_HEIGHT; // Goal height (m)

        // Calculate horizontal distance from the robot to the goal
        double d = fromPose.getTranslation().getNorm(); // Horizontal distance (m)

        // Dynamic Horizontal Score Offset
        double p = d / 2; // A chosen offset, can be adjusted based on empirical data

        // Calculate angle of launch
        double angleRadians = Math.atan((-2 * d * h0 + 2 * d * hg - h0 * p + hg * p) / (Math.pow(d, 2) + d * p));
        return Math.toDegrees(angleRadians); // Convert to degrees
    }


    public double calculateSpeed(double atAngle, Pose2d fromPose){
        // Constants
        double g = -9.81; // Gravity (m/s^2)
        double h0 = ROBOT_SHOOTER_HEIGHT; // Shooter height (m)
        double hg = GOAL_HEIGHT; // Goal height (m)

        // Calculate horizontal distance from the robot to the goal
        double d = fromPose.getTranslation().getNorm(); // Horizontal distance (m)
        double angleRadians = Math.toRadians(atAngle); // Convert angle to radians

        // Use the formula for launch speed derived from projectile motion equations
        double launchSpeed = (d * Math.sqrt(-g / 2) * sec(angleRadians)) /
                Math.sqrt(-(hg - d * Math.tan(angleRadians) - h0));

        return launchSpeed;
    }

    // Helper method to calculate secant, as Java Math does not directly provide it
    private double sec(double angleRadians) {
        return 1 / Math.cos(angleRadians);
    }

    // Calculate the distance and shoot with appropriate speed regardless of alignment
    public void shootEstimate(Pose2d fromPose){
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
        double angle = getEstimatedShootingAngle(horizontalDistance, exitVelocity);

        setAngle(angle);
        shooterMotor.set(speed);
        isShooting = true;
    }

    private static double getEstimatedShootingAngle(double horizontalDistance, double exitVelocity) {
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

        if (isShooting && Math.abs(getAngle() - goalRotation/SHOOTER_ANGLE_CONVERSION) < SHOOTING_ANGLE_ERROR) {
            if (shootDelayCounter < shootDelay) {
                shootDelayCounter += dt;
                shooterMotor.set(1.0);
            } else if (shootDelayCounter < shootEndDelay) {
                shootDelayCounter += dt;
                loadingMotor.set(0.5);
            } else {
                System.out.println("Finished Shooting");
                intakeMotor.stopMotor();
                loadingMotor.stopMotor();
                shooterMotor.stopMotor();
                isShooting = false;
                finishedShooting = true;
            }
        } else {
            if(isLoaded()) {
                shooterMotor.stopMotor();
                intakeMotor.stopMotor();
                loadingMotor.stopMotor();
            }
        }

        if(isLowestAngle()){
            angleAlignmentMotor.stopMotor();
            // angleEncoder.setPosition(0);

            if (goalRotation < 0) {
                setRotation(0);
            }
        }else if(isHighestAngle()){
            goalRotation = Math.min(goalRotation, angleEncoder.getPosition());
        }

        double angleMoveSpeed = anglePIDController.calculate(getAngle(), goalRotation);
        if((angleMoveSpeed > 0 && !isHighestAngle()) || (angleMoveSpeed < 0 && !isLowestAngle())) {
            angleAlignmentMotor.set(-angleMoveSpeed/40);
        }

        updateShuffleboard();
    }

    public double getAngle(){
        return angleEncoder.getPosition() / SHOOTER_ANGLE_CONVERSION + SHOOTER_RESTING_ANGLE;
    }

    public void setLaunchSpeed(double speed){
        shooterMotor.set(speed * LAUNCH_SPEED_CONVERSION);
    }

    public boolean isLoaded(){
        // return false;
        return !isLoadedButton.get();
    }

    public boolean isLowestAngle(){
        return !isLowestAngleButton.get();
    }

    public boolean isHighestAngle(){
        return !isHighestAngleButton.get();
    }
}
