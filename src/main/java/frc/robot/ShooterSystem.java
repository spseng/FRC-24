package frc.robot;

import com.revrobotics.AlternateEncoderType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


import com.ctre.phoenix6.hardware.CANcoder;
import com.fasterxml.jackson.databind.AnnotationIntrospector.ReferenceProperty.Type;

import static frc.robot.Constants.*;

public class ShooterSystem {

    // CANVenom intakeMotor = new CANVenom()
    // private final Spark intakeMotor;
    // private final Spark shooterMotor;

    private final CANSparkMax intakeMotor;
    private final CANSparkMax loadingMotor;
    private final CANSparkMax shooterMotor;
    private final CANSparkMax angleAlignmentMotor;

    private final CANcoder angleEncoder;
    
    // private final WPI_CANCoder angleEncoder;
    // private final WPI_TalonFX angleAlignmentMotor;

    private final PIDController anglePIDController = new PIDController(SHOOTING_ANGLE_KP, SHOOTING_ANGLE_KI, SHOOTING_ANGLE_KD);
    private final PIDController _shooterController = new PIDController(SHOOTER_KP, SHOOTER_KI, SHOOTER_KD);


    private double shootEndDelay = 0.6;
    private double shootDelay = 0.2;
    private double shootDelayCounter = 0;
    private double angleMoveSpeed;


    private boolean isShooting = false;
    private boolean finishedShooting = false; // TODO: Find better way to tell if auton shot is done
//    private boolean isCalibrating = false;
    private double goalRotation;

    private final DigitalInput isLoadedButton;
    private final DigitalInput isLowestAngleButton;
    private final DigitalInput isHighestAngleButton;

    public ShooterSystem(int intakeMotorCAN, int loadingMotorCAN, int shooterMotorCAN, int angleAlignmentMotorCAN, int angleAlignmentEncoderCAN, int isLoadedButtonChannel, int isLowestAngleButtonChannel, int isHighestAngleButtonChannel) {
        intakeMotor = new CANSparkMax(intakeMotorCAN, MotorType.kBrushless);
        intakeMotor.setInverted(true);

        loadingMotor = new CANSparkMax(loadingMotorCAN, MotorType.kBrushless);
        loadingMotor.setInverted(true);

        shooterMotor = new CANSparkMax(shooterMotorCAN, MotorType.kBrushless);
        shooterMotor.setInverted(true);


        angleAlignmentMotor = new CANSparkMax(angleAlignmentMotorCAN, MotorType.kBrushed);
        angleEncoder = new CANcoder(ARM_CANCODER);
        

        isLoadedButton = new DigitalInput(isLoadedButtonChannel);
        isLowestAngleButton = new DigitalInput(isLowestAngleButtonChannel);
        isHighestAngleButton = new DigitalInput(isHighestAngleButtonChannel);

    }

    public void updateShuffleboard() {
        SmartDashboard.putBoolean("IntakeLoaded", isLoaded());
        SmartDashboard.putBoolean("Is Lowest", isLowestAngle());
        SmartDashboard.putBoolean("Is Highest", isHighestAngle());

        SmartDashboard.putNumber("Angle", getAngle());
        SmartDashboard.putNumber("Goal Angle", getGoalAngle());
        SmartDashboard.putNumber("motor out", getMotorOutput());

        SmartDashboard.putNumber("Shooter Counter", shootDelayCounter);
    }

    public void calibrate(){
        if(isLowestAngle()){
            angleAlignmentMotor.stopMotor();
          
        }
    }

    public void setArmAngle(double angle){
        goalRotation = angle;
    }

    public void setRotation(double angle){
        goalRotation = angle;
    }

    public void intakeUnlessLoaded(){
        if(!isLoaded()){
            intakeMotor.set(INTAKE_SPEED);
            if(isLowestAngle()){
                loadingMotor.set(LOADING_SPEED);
            }
            // shooterMotor.set(-INTAKE_SPEED/50);
        } else {
            intakeMotor.stopMotor();
            loadingMotor.stopMotor();
            shooterMotor.stopMotor();
        }
    }

    public void rotateAngle(double amount) {
        if((amount > 0 && !isHighestAngle()) || (amount < 0 && !isLowestAngle())) {
            goalRotation += amount;
        }
    }

    // public void spinAngle() {
    //     angleAlignmentMotor.set(-0.2);
    // }

    public void rejectCurrentIntake(){
        System.out.println("Rejecting");
        intakeMotor.set(-0.8);
        loadingMotor.set(-0.3);
        shooterMotor.set(-0.8);
    }

    public void stopIntake() {
        if(!isShooting){
            intakeMotor.stopMotor();
            loadingMotor.stopMotor();
            shooterMotor.stopMotor();
        }
    }

    public void stopAngleAlignment(){
        angleAlignmentMotor.stopMotor();
        
    }

    public void shootMaxSpeed(){

        shooterMotor.set(SHOOT_STATIC_SPEED);
        isShooting = true;
        shootDelayCounter = 0.0;
    }

    public void shootAmp(){
        setArmAngle(AMP_SCORING_ANGLE);
        shooterMotor.set(SHOOT_STATIC_SPEED);
        isShooting = true;
        shootDelayCounter = 0.0;
    }

    // public void shootPose(Pose2d fromPose){
    //     double calculatedAngle = calculateAngle(fromPose);
    //     setArmAngle(calculatedAngle);
    //     shooterMotor.set(calculateSpeed(calculatedAngle, fromPose));
    //     isShooting = true;
    //     shootDelayCounter = 0.0;
    // }

    // public void lineUpAngle(Pose2d fromPose){
    //     double angle = calculateAngle(fromPose);
    //     setArmAngle(angle);
    // }

    // public double calculateAngle(Pose2d fromPose){
    //     // Constants
    //     double g = -9.81; // Gravity (m/s^2)
    //     double h0 = ROBOT_SHOOTER_HEIGHT; // Shooter height (m)
    //     double hg = GOAL_HEIGHT; // Goal height (m)

    //     // Calculate horizontal distance from the robot to the goal
    //     double d = fromPose.getTranslation().getNorm(); // Horizontal distance (m)

    //     // Dynamic Horizontal Score Offset
    //     double p = d / 2; // A chosen offset, can be adjusted based on empirical data

    //     // Calculate angle of launch
    //     double angleRadians = Math.atan((-2 * d * h0 + 2 * d * hg - h0 * p + hg * p) / (Math.pow(d, 2) + d * p));
    //     return Math.toDegrees(angleRadians); // Convert to degrees
    // }


    // public double calculateSpeed(double atAngle, Pose2d fromPose){
    //     // Constants
    //     double g = -9.81; // Gravity (m/s^2)
    //     double h0 = ROBOT_SHOOTER_HEIGHT; // Shooter height (m)
    //     double hg = GOAL_HEIGHT; // Goal height (m)

    //     // Calculate horizontal distance from the robot to the goal
    //     double d = fromPose.getTranslation().getNorm(); // Horizontal distance (m)
    //     double angleRadians = Math.toRadians(atAngle); // Convert angle to radians

    //     // Use the formula for launch speed derived from projectile motion equations
    //     double launchSpeed = (d * Math.sqrt(-g / 2) * sec(angleRadians)) /
    //             Math.sqrt(-(hg - d * Math.tan(angleRadians) - h0));

    //     return launchSpeed;
    // }

    // // Helper method to calculate secant, as Java Math does not directly provide it
    // private double sec(double angleRadians) {
    //     return 1 / Math.cos(angleRadians);
    // }

    // public boolean autonShoot(Pose2d pose) {
    //     if(!isShooting) {
    //         shootPose(pose);
    //     } else return finishedShooting;

    //     return false;
    // }

    // public boolean autonShootAmp() {
    //     if(!isShooting) {
    //         shootAmp();
    //     } else {
    //         return finishedShooting;
    //     }

    //     return false;
    // }

    public void periodic(double dt) {
        finishedShooting = false;

        
        if (isShooting && Math.abs(getAngle() - getGoalAngle()) < SHOOTING_ANGLE_ERROR) {
            if (shootDelayCounter < shootDelay) {
                shootDelayCounter += dt;
                shooterMotor.set(SHOOT_STATIC_SPEED);
            } else if (shootDelayCounter < shootEndDelay) {
                shootDelayCounter += dt;
                loadingMotor.set(SHOOT_STATIC_SPEED);
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

            
        }else if(isHighestAngle()){
            angleAlignmentMotor.stopMotor();
            // goalRotation = Math.min(goalRotation, angleEncoder.getAbsolutePosition().getValueAsDouble());
        }

        angleMoveSpeed = anglePIDController.calculate(getAngle(), getGoalAngle());
        // if (goalRotation > Constants.ARM_MAX_ANGLE){
        //     setRotation(Constants.ARM_MAX_ANGLE);
        // }else if (goalRotation < Constants.ARM_INTAKE_ANGLE){
        //     setRotation(Constants.ARM_INTAKE_ANGLE);
        // }
        if((angleMoveSpeed > 0 && !isHighestAngle()) || (angleMoveSpeed < 0 && !isLowestAngle())) {
            angleAlignmentMotor.set(-angleMoveSpeed);
        }

        updateShuffleboard();
    }

    public double getAngle(){
        return angleEncoder.getAbsolutePosition().getValueAsDouble() ;
    }

    public double getMotorOutput(){
        return angleMoveSpeed;
    }

    public double getGoalAngle(){
        return -goalRotation;
    }

    public boolean isLoaded(){
        // return false;
        return !isLoadedButton.get();
    }

    public boolean isLowestAngle(){
        return !isLowestAngleButton.get();
    }
    public void setToAmp(){
        setArmAngle(AMP_SCORING_ANGLE);
    }
    public boolean isHighestAngle(){
        return !isHighestAngleButton.get();
    }
}
