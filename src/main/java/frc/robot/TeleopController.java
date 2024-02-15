package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import static frc.robot.Constants.*;

public class TeleopController {
    // Instance Variables
    boolean joystickController = false;

    double goalHeading = 0;

    // private final StructArrayPublisher<SwerveModuleState> publisherReal;
    // private final StructArrayPublisher<SwerveModuleState> publisherGoal;
    // private final StructPublisher<Pose2d> publisherPose;

    public TeleopController(boolean jsController) {
        joystickController = jsController;

        // publisherReal = NetworkTableInstance.getDefault().getStructArrayTopic("/SwerveStates", SwerveModuleState.struct).publish();
        // publisherGoal = NetworkTableInstance.getDefault().getStructArrayTopic("/SwerveStatesGoal", SwerveModuleState.struct).publish();
        // publisherPose = NetworkTableInstance.getDefault().getStructTopic("/RoboPose", Pose2d.struct).publish();
    }

    public void teleopInit(Drivetrain driveTrain) {
        // Calibrate relative encoders to match absolute encoders
        driveTrain.calibrateSteering();
    }

    public void teleopPeriodic(XboxController m_stick, Drivetrain drivetrain, VisionController visionController) {
        // double leftX = Math.abs(m_stick.getLeftX()) < JOYSTICK_DEAD_ZONE ? 0 : -m_stick.getLeftX();
        // double leftY = Math.abs(m_stick.getLeftY()) < JOYSTICK_DEAD_ZONE ? 0 : -m_stick.getLeftY();
        // double leftAngle = getDriveAngle(leftX, leftY);

        // double leftR = Math.sqrt(Math.pow(leftX, 2) + Math.pow(leftY, 2));
        // double driveSpeed = (leftR < JOYSTICK_DEAD_ZONE) ? 0 : leftR * DRIVE_SPEED;

        // double rightX = -m_stick.getRightX();
        // double rightY = m_stick.getRightY();
        // double rightR = Math.sqrt(Math.pow(rightX, 2) + Math.pow(rightY, 2));
        // double rightAngle = getHeadingAngle(rightX, rightY);

        // if (rightR > JOYSTICK_DEAD_ZONE){
        //     goalHeading += rightX * TURN_SPEED;
        //     // goalHeading = goalHeading % FULL_ROTATION;
        // }

        // // // Check if either joystick is beyond the dead zone
        // if (driveSpeed > 0) {
        //     drivetrain.move(leftX, leftY, goalHeading); // Using Odometry
        // } else
        // // if(rightR > JOYSTICK_DEAD_ZONE){

        // // } else
        // if (m_stick.getAButton()) {
        //     drivetrain.steer(0);
        // } else
        // if (m_stick.getBButton()) {
        //     goalHeading += FULL_ROTATION * 0.25;
        //     // goalHeading = goalHeading % FULL_ROTATION;
        // } else
        // if (m_stick.getXButton()) {
        //     drivetrain.pointStraight();
        // } else
        // if (m_stick.getYButton()){
        //     drivetrain.moveTo(0, 0, 0);
        // } else
        // if (m_stick.getRightBumperReleased()) {
        //     goalHeading = 0;
        //     drivetrain.calibrateSteering();
        // } else {
        //     drivetrain.move(0, 0, goalHeading);
        // }

        // drivetrain.periodic();

        // drivetrain.updateShuffleboard();

        // publisherGoal.set(drivetrain.getGoalSwerveModuleStates());
        // publisherReal.set(drivetrain.getRealSwerveModuleStates());
        // publisherPose.set(drivetrain.getPose());
    }

    // Helper functions
    public double getDriveAngle(double x, double y) {
        return (((Math.atan2(y, -x))/Math.PI / 2 * FULL_ROTATION + 0.0*FULL_ROTATION) % FULL_ROTATION);
    }
    public double getHeadingAngle(double x, double y) {
        return (((Math.atan2(y, -x))/Math.PI / 2 * FULL_ROTATION + 0.25*FULL_ROTATION) % FULL_ROTATION);
    }
}
