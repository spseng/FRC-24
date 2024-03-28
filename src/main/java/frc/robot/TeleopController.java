package frc.robot;

import static java.lang.Math.abs;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Commands.SetArmAngleAmp;
import frc.robot.Commands.SetArmAngleIntake;
import frc.robot.Commands.Shoot;
import frc.robot.drive.Drivetrain;
import frc.robot.vision.FieldLayout;
import frc.robot.vision.VisionSystem;

import static frc.robot.Constants.*;

public class TeleopController {
    // Instance Variables
    boolean joystickController = false;
    boolean isLeftTriggerActive = false;
    

    private final StructArrayPublisher<SwerveModuleState> publisherReal;
    private final StructArrayPublisher<SwerveModuleState> publisherGoal;
    private final StructPublisher<Pose2d> publisherPose;
    
    public TeleopController(boolean jsController) {
        joystickController = jsController;

        publisherReal = NetworkTableInstance.getDefault().getStructArrayTopic("/SwerveStates", SwerveModuleState.struct).publish();
        publisherGoal = NetworkTableInstance.getDefault().getStructArrayTopic("/SwerveStatesGoal", SwerveModuleState.struct).publish();
        publisherPose = NetworkTableInstance.getDefault().getStructTopic("/RoboPose", Pose2d.struct).publish();
    }

    public void init(Drivetrain driveTrain) {
        // Calibrate relative encoders to match absolute encoders
        driveTrain.calibrateSteering();
    }


    /*
        Initial test todo!
        • Have the bumpers run the two stages in reverse at max speed.
            • Left Bumper runs intake and loader backwards
            • Right Bumper runs shooter backwards
        •
     */

    /*
        Left Stick: Swerve Drive
        Right Stick: Rotate

        Left Trigger: Intake
        Left Bumper: Reject Intake and Roller
        Right Trigger: Line up shot
        Right Bumper: Shoot

        DPad Up:
     */

    public void periodic(XboxController m_stick, Drivetrain drivetrain, ShooterSystem shooterSystem) {
        double leftX = Math.abs(m_stick.getLeftX()) < JOYSTICK_DEAD_ZONE ? 0 : -m_stick.getLeftX();
        double leftY = Math.abs(m_stick.getLeftY()) < JOYSTICK_DEAD_ZONE ? 0 : -m_stick.getLeftY();

        double leftR = Math.sqrt(Math.pow(leftX, 2) + Math.pow(leftY, 2));
        double driveSpeed = (leftR < JOYSTICK_DEAD_ZONE) ? 0 : leftR * DRIVE_SPEED;

        double rightX = -m_stick.getRightX();
        double rightY = m_stick.getRightY();
        double rightR = Math.sqrt(Math.pow(rightX, 2) + Math.pow(rightY, 2));

        if (rightR > JOYSTICK_DEAD_ZONE) {
            drivetrain.rotate(-rightX);
        }
        else {
            drivetrain.rotate(0);
        }

        // Check if either joystick is beyond the dead zone
        if (driveSpeed > 0) {
            drivetrain.move(leftX, leftY);
        }  else if (m_stick.getRightTriggerAxis() > TRIGGER_DEAD_ZONE) {
            
          shooterSystem.shootMaxSpeed();
        } else {
            drivetrain.move();
        }
        
        if (m_stick.getLeftTriggerAxis() > TRIGGER_DEAD_ZONE) {
            if(!isLeftTriggerActive){
                shooterSystem.setArmAngle(0);
                shooterSystem.intakeUnlessLoaded();
            }
            isLeftTriggerActive = true;
        }else if(m_stick.getLeftTriggerAxis() < TRIGGER_DEAD_ZONE){
            if(isLeftTriggerActive){
                shooterSystem.stopIntake();
            }
            isLeftTriggerActive = false;
        }
        
        if (m_stick.getLeftBumper()) {
            shooterSystem.rejectCurrentIntake();
        } else if (m_stick.getRightBumper()) {
           shooterSystem.setArmAngle(Constants.AMP_SCORING_ANGLE);
        } else if (m_stick.getBButton()) {
          shooterSystem.setArmAngle(Constants.ARM_INTAKE_ANGLE);
        } else if (m_stick.getXButton()) {
            drivetrain.calibrateSteering();
        } else if (m_stick.getYButton()) {
            shooterSystem.setArmAngle(Constants.AMP_SCORING_ANGLE);
        }

        if (m_stick.getAButton()) {
            shooterSystem.stopAngleAlignment();
        } else if (m_stick.getPOV() == 0) {
            shooterSystem.rotateAngle(1);
        } else if (m_stick.getPOV() == 180) {
            shooterSystem.rotateAngle(-1);
        } else if(m_stick.getPOV() == -1) {
            shooterSystem.stopAngleAlignment();
        }

        if (m_stick.getLeftTriggerAxis() < TRIGGER_DEAD_ZONE && !m_stick.getLeftBumper() ) {
            shooterSystem.stopIntake();
        }

//         publisherGoal.set(drivetrain.getGoalSwerveModuleStates());
//         publisherReal.set(drivetrain.getRealSwerveModuleStates());
//         publisherPose.set(drivetrain.getPose());
    }
}
