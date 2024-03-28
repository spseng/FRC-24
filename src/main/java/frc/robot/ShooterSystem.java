package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


import com.ctre.phoenix6.hardware.CANcoder;

import static frc.robot.Constants.*;

public class ShooterSystem {
    private final CANSparkMax intakeMotor;
    private final CANSparkMax loadingMotor;
    private final CANSparkMax shooterMotor;
    private final CANSparkMax angleAlignmentMotor;
    private final CANcoder angleEncoder;


    private final PIDController anglePIDController = new PIDController(SHOOTING_ANGLE_KP, SHOOTING_ANGLE_KI, SHOOTING_ANGLE_KD);
    private final PIDController _shooterController = new PIDController(SHOOTER_KP, SHOOTER_KI, SHOOTER_KD);

    private double shootEndDelay = 0.6;
    private double shootDelay = 0.2;
    private double shootDelayCounter = 0;

    private double angleMoveSpeed = 0;

    private boolean isShooting = false;
    private boolean finishedShooting = false; // TODO: Find better way to tell if auton shot is done

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

        SmartDashboard.putNumber("Angle", getEncoderPosition());
        SmartDashboard.putNumber("Goal Angle", getGoalRotation());
        SmartDashboard.putNumber("motor out", getMotorOutput());

        SmartDashboard.putNumber("Shooter Counter", shootDelayCounter);
    }

    public void calibrate(){
        if(isLowestAngle()){
            angleAlignmentMotor.stopMotor();
          
        }
    }

    public void setArmRotation(double angle){
        goalRotation = angle;
    }

    public void intakeUnlessLoaded(){
        if(!isLoaded()){
            intakeMotor.set(INTAKE_SPEED);
            if(isLowestAngle()){
                loadingMotor.set(LOADING_SPEED);
            }
        } else {
            intakeMotor.stopMotor();
            loadingMotor.stopMotor();
            shooterMotor.stopMotor();
        }
    }

    public void rotateArmAngle(double amount) {
        if((amount > 0 && !isHighestAngle()) || (amount < 0 && !isLowestAngle())) {
            goalRotation = Math.min(Math.max(goalRotation + amount, ARM_MAX_ANGLE), ARM_INTAKE_ANGLE);
        }
    }

    public void rejectCurrentIntake(){
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
        setArmRotation(getEncoderPosition());
    }

    public void shootMaxSpeed(){
        shooterMotor.set(SHOOT_STATIC_SPEED);
        isShooting = true;
        shootDelayCounter = 0.0;
    }

    public void shootAmp(){
        setArmRotation(AMP_SCORING_ANGLE);
        shooterMotor.set(SHOOT_STATIC_SPEED);
        isShooting = true;
        shootDelayCounter = 0.0;
    }

    public void periodic(double dt) {
        finishedShooting = false;

        if (isShooting && Math.abs(getEncoderPosition() - getGoalRotation()) < SHOOTING_ANGLE_ERROR) {
            if (shootDelayCounter < shootDelay) {
                shootDelayCounter += dt;
                shooterMotor.set(SHOOT_STATIC_SPEED);
            } else if (shootDelayCounter < shootEndDelay) {
                shootDelayCounter += dt;
                loadingMotor.set(SHOOT_STATIC_SPEED);
            } else {
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

        angleMoveSpeed = Math.max(Math.min(anglePIDController.calculate(getEncoderPosition(), getGoalRotation()), 1), -1);

        if (isLowestAngle() && angleMoveSpeed > 0) {
            setArmRotation(getEncoderPosition());
            angleMoveSpeed = 0;
        }

        if (isHighestAngle() && angleMoveSpeed < 0) {
            setArmRotation(getEncoderPosition());
            angleMoveSpeed = 0;
        }

        angleAlignmentMotor.set(angleMoveSpeed);

        updateShuffleboard();
    }

    public double getEncoderPosition(){
        return angleEncoder.getAbsolutePosition().getValueAsDouble() ;
    }

    public double getMotorOutput(){
        return angleMoveSpeed;
    }

    public double getGoalRotation(){
        return goalRotation;
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
