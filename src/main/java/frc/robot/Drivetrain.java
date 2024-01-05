package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import static frc.robot.Constants.*;
import static frc.robot.Constants.BR_STEER_OFFSET;

public class Drivetrain {
    public static final double kMaxSpeed = 3.0; // 3 meters per second
    public static final double kMaxAngularSpeed = Math.PI; // 1/2 rotation per second



    // Locations for the swerve drive modules relative to the robot center.
    private final Translation2d m_frontLeftLocation = new Translation2d(0.381, 0.381);
    private final Translation2d m_frontRightLocation = new Translation2d(0.381, -0.381);
    private final Translation2d m_backLeftLocation = new Translation2d(-0.381, 0.381);
    private final Translation2d m_backRightLocation = new Translation2d(-0.381, -0.381);

    private final SwerveMotor br_motor;
    private final SwerveMotor fr_motor;
    private final SwerveMotor fl_motor;
    private final SwerveMotor bl_motor;


    private final AnalogGyro m_gyro = new AnalogGyro(0);

    // Creating my kinematics object using the module locations
    private SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
            m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation
    );

    // Creating my odometry object from the kinematics object and the initial wheel positions.
// Here, our starting pose is 5 meters along the long end of the field and in the
// center of the field along the short end, facing the opposing alliance wall.
    private final SwerveDriveOdometry m_odometry;

    public Drivetrain() {
        br_motor = new SwerveMotor(BR_STEER_CAN, BR_DRIVE_CAN, BR_STEER_OFFSET);
        fr_motor = new SwerveMotor(FR_STEER_CAN, FR_DRIVE_CAN, FR_STEER_OFFSET);
        fl_motor = new SwerveMotor(FL_STEER_CAN, FL_DRIVE_CAN, FL_STEER_OFFSET);
        bl_motor = new SwerveMotor(BL_STEER_CAN, BL_DRIVE_CAN, BL_STEER_OFFSET);

        m_odometry = new SwerveDriveOdometry(
                m_kinematics, m_gyro.getRotation2d(),
                new SwerveModulePosition[] {
                        br_motor.getSwervePosition(),
                        fr_motor.getSwervePosition(),
                        fl_motor.getSwervePosition(),
                        bl_motor.getSwervePosition()
                }, new Pose2d(5.0, 13.5, new Rotation2d()));

        m_gyro.reset();
    }

    public void periodic() {
        // Get the rotation of the robot from the gyro.
        var gyroAngle = m_gyro.getRotation2d();

        // Update the pose
        m_odometry.update(
            m_gyro.getRotation2d(),
            new SwerveModulePosition[] {
                    br_motor.getSwervePosition(),
                    fr_motor.getSwervePosition(),
                    fl_motor.getSwervePosition(),
                    bl_motor.getSwervePosition()
            }
        );
    }



    public void updateShuffleboard() {
        SmartDashboard.putNumber("BR Position", br_motor.getSteeringPosition());
        SmartDashboard.putNumber("FR Position", fr_motor.getSteeringPosition());
        SmartDashboard.putNumber("FL Position", fl_motor.getSteeringPosition());
        SmartDashboard.putNumber("BL Position", bl_motor.getSteeringPosition());

        SmartDashboard.putNumber("BR ABS Position", br_motor.getAbsoluteSteeringPosition());
        SmartDashboard.putNumber("FR ABS Position", fr_motor.getAbsoluteSteeringPosition());
        SmartDashboard.putNumber("FL ABS Position", fl_motor.getAbsoluteSteeringPosition());
        SmartDashboard.putNumber("BL ABS Position", bl_motor.getAbsoluteSteeringPosition());
    }

    // Motor functions
    public void calibrate(){
        br_motor.calibrate();
        fr_motor.calibrate();
        bl_motor.calibrate();
        fl_motor.calibrate();
    }

    public void zeroSteering() {
        br_motor.zeroPosition();
        fr_motor.zeroPosition();
        bl_motor.zeroPosition();
        fl_motor.zeroPosition();
    }

    public void stopSteering() {
        br_motor.stopSteering();
        fr_motor.stopSteering();
        bl_motor.stopSteering();
        fl_motor.stopSteering();
    }

    public void steer(double theta) {
        br_motor.steer(theta);
        fr_motor.steer(theta);
        bl_motor.steer(theta);
        fl_motor.steer(theta);
    }

    public void drive(double r) {
        br_motor.drive(r);
        fr_motor.drive(r);
        bl_motor.drive(r);
        fl_motor.drive(r);
    }

    public void turn(double r, double directionFactor) {
        br_motor.steer(-0.25);
        fr_motor.steer(0.25);
        bl_motor.steer(0.25);
        fl_motor.steer(-0.25);

        br_motor.drive(r * directionFactor);
        fr_motor.drive(r * directionFactor);
        bl_motor.drive(r * directionFactor);
        fl_motor.drive(r * directionFactor);
    }
}
