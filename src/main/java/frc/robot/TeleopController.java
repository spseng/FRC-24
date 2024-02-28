package frc.robot;
import static java.lang.Math.abs;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.drive.Drivetrain;
import frc.robot.vision.FieldLayout;
import frc.robot.vision.VisionSystem;

import static frc.robot.Constants.*;

public class TeleopController {
    // Instance Variables
    boolean joystickController = false;

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
        Left Stick: Swerve Drive
        Right Stick: Rotate

        Left Bumper: Reject current intake
        Right Bumper: Shoot
        Right Trigger: Hold to line up shot with goal
        Left Trigger: Intake unless loaded

        DPad Up: Hold to Move towards Nearest Amp
        DPad Left: Hold to Move towards Nearest Goal
        DPad Right: Hold to Move towards Nearest Loading Station
     */

    public void periodic(XboxController m_stick, Drivetrain drivetrain, ShooterSystem shooterSystem, VisionSystem visionSystem) {
         double leftX = Math.abs(m_stick.getLeftX()) < JOYSTICK_DEAD_ZONE ? 0 : -m_stick.getLeftX();
         double leftY = Math.abs(m_stick.getLeftY()) < JOYSTICK_DEAD_ZONE ? 0 : -m_stick.getLeftY();

         double leftR = Math.sqrt(Math.pow(leftX, 2) + Math.pow(leftY, 2));
         double driveSpeed = (leftR < JOYSTICK_DEAD_ZONE) ? 0 : leftR * DRIVE_SPEED;

         double rightX = -m_stick.getRightX();
         double rightY = m_stick.getRightY();
         double rightR = Math.sqrt(Math.pow(rightX, 2) + Math.pow(rightY, 2));

         if (rightR > JOYSTICK_DEAD_ZONE){
             drivetrain.rotate(rightX);
         }

         // Check if either joystick is beyond the dead zone
         if (driveSpeed > 0) {
             drivetrain.move(leftX, leftY); // Using Odometry
         } else
        if (m_stick.getAButton()) {
            shooterSystem.intakeUnlessLoaded();
        }else if(m_stick.getLeftTriggerAxis() > TRIGGER_DEAD_ZONE) {

            // Line up shot with goal
            Pose2d robotPose = drivetrain.getPose();
            Pose2d nearestGoal = FieldLayout.getGoalGoal(robotPose);
            drivetrain.pointTowards(nearestGoal);
            shooterSystem.lineUpAngle(robotPose);

            drivetrain.move();
        }else if(m_stick.getRightTriggerAxis() > TRIGGER_DEAD_ZONE) {
            shooterSystem.shoot(drivetrain.getPose());
        }else if(m_stick.getLeftBumper()) {
            shooterSystem.rejectCurrentIntake();
        } else if (m_stick.getLeftBumper()) { // Points towards nearest april tag // Testing
            double measuredYaw = visionSystem.getTargetRelativeYaw();
            if (abs(measuredYaw) > 0.1) {
                drivetrain.setYawHeadingOffset(measuredYaw);
            }
        }else if (m_stick.getLeftBumperReleased()) {
            drivetrain.setYawHeadingOffset(0);
        }else if (m_stick.getBButton()) {
             drivetrain.rotate(FULL_ROTATION * 0.25 / TURN_SPEED);
        } else if (m_stick.getYButton()) {
            drivetrain.moveTo(0, 0, 0);
        } else if(m_stick.getBButton()){
            drivetrain.moveTo("amp");
        } else {
            drivetrain.move();
        }

//         publisherGoal.set(drivetrain.getGoalSwerveModuleStates());
//         publisherReal.set(drivetrain.getRealSwerveModuleStates());
//         publisherPose.set(drivetrain.getPose());
    }
}
