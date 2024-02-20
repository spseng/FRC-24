package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.vision.AprilTagFieldLayout;

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
//    private boolean isCalibrating = false;
    private double goalAngle = 0.0;

    private final DigitalInput isLoadedButton;

    private final DigitalInput isLowestAngleButton;
    private final DigitalInput isHighestAngleButton;

    public ShooterSystem(int intakeMotorCAN, int loadingMotorCAN, int shooterMotorCAN, int angleAlignmentMotorCAN, int isLoadedButtonChannel, int isLowestAngleButtonChannel, int isHighestAngleButtonChannel) {
        // intakeMotor = new Spark(intakeMotorChannel);
        // shooterMotor = new Spark(shooterMotorChannel);
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
        SmartDashboard.putNumber("Goal Angle", goalAngle);
    }

    public void calibrate(){
        if(isLowestAngle()){
            angleAlignmentMotor.stopMotor();
            angleEncoder.setPosition(0);
        }
        setAngle(0);
    }


    public void setAngle(double angle){
        goalAngle = Math.max(angle, 0);
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

    public Pose2d getGoalGoal(Pose2d fromPosition){
        // Page 4 of https://firstfrc.blob.core.windows.net/frc2024/FieldAssets/2024LayoutMarkingDiagram.pdf
        // Return either tag id #4 if red or #7 if blue alliance
        return fromPosition.nearest(AprilTagFieldLayout.goalPositions);
//        return AprilTagFieldLayout.getTagPose(4);
//        return AprilTagFieldLayout.getTagPose(7);
    }

    // Calculate the distance and shoot with appropriate speed regardless of alignment
    public void shoot(Pose2d fromPose){
        double distance = fromPose.getTranslation().getNorm();
        SmartDashboard.putNumber("Shot from", distance);

        double speed = 0.0;

        // TODO: Replace this with a continuous function
        // TODO: Test these rough values
        if(distance < 1.0){
            speed = 0.5;
        } else if(distance < 2.0){
            speed = 0.6;
        } else if(distance < 3.0){
            speed = 0.7;
        } else if(distance < 4.0){
            speed = 0.8;
        } else if(distance < 5.0){
            speed = 0.9;
        } else {
            speed = 1.0;
        }



        shooterMotor.set(speed);
        isShooting = true;
    }

    public void periodic(double dt) {
        if (isShooting) {
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
            }
        }

        double angleMoveSpeed = anglePIDController.calculate(angleEncoder.getPosition(), goalAngle);
        angleAlignmentMotor.set(angleMoveSpeed);

        if(isLowestAngle()){
            angleAlignmentMotor.stopMotor();
            angleEncoder.setPosition(0);

            if (goalAngle < 0) {
                setAngle(0);
            }
        }

        if(isHighestAngle()){
            goalAngle = Math.min(goalAngle, angleEncoder.getPosition());
        }

        updateShuffleboard();
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
