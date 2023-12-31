package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.AnalogGyro;

//import com.kauailabs.navx.frc.*;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import static frc.robot.Constants.*;

public class Drivetrain {

    // Locations for the swerve drive modules relative to the robot center (pretty sure it's based off CoM)
    private final Translation2d br_location = new Translation2d(-0.381, -0.381); // Arbitrary values
    private final Translation2d fr_location = new Translation2d(0.381, -0.381); // Arbitrary values
    private final Translation2d fl_location = new Translation2d(0.381, 0.381); // Arbitrary values
    private final Translation2d bl_location = new Translation2d(-0.381, 0.381); // Arbitrary values

    // Creating my kinematics object using the module locations
    private final SwerveDriveKinematics driveKinematics = new SwerveDriveKinematics(
            fl_location, fr_location, bl_location, br_location
    );


    private final SwerveMotor br_motor;
    private final SwerveMotor fr_motor;
    private final SwerveMotor fl_motor;
    private final SwerveMotor bl_motor;


    // Turning
    private final PIDController turningPIDController;
    private final AnalogGyro m_gyro = new AnalogGyro(0); // Placeholder for the navX gyro
//    private final AHRS gyro = new AHRS(SPI.Port.kMXP);  // navX gyro

    // Creating my odometry object from the kinematics object and the initial wheel positions.
// Here, our starting pose is 5 meters along the long end of the field and in the
// center of the field along the short end, facing the opposing alliance wall.
    private final SwerveDriveOdometry odometry;

    public Drivetrain() {
        br_motor = new SwerveMotor(BR_STEER_CAN, BR_DRIVE_CAN, BR_STEER_OFFSET);
        fr_motor = new SwerveMotor(FR_STEER_CAN, FR_DRIVE_CAN, FR_STEER_OFFSET);
        fl_motor = new SwerveMotor(FL_STEER_CAN, FL_DRIVE_CAN, FL_STEER_OFFSET);
        bl_motor = new SwerveMotor(BL_STEER_CAN, BL_DRIVE_CAN, BL_STEER_OFFSET);

        odometry = new SwerveDriveOdometry(
                driveKinematics, m_gyro.getRotation2d(),
//                driveKinematics, gyro.getYaw(), // Used with navX gyro
                new SwerveModulePosition[] {
                        br_motor.getSwervePosition(),
                        fr_motor.getSwervePosition(),
                        fl_motor.getSwervePosition(),
                        bl_motor.getSwervePosition()
                }, new Pose2d(0, 0, new Rotation2d()));

        this.turningPIDController = new PIDController(TURNING_KP, TURNING_KI, TURNING_KD);

        m_gyro.reset();
    }

    public void periodic() {
        // Get the rotation of the robot from the gyro.
        var gyroAngle = m_gyro.getRotation2d();
//        var gyroAngle = gyro.getYaw(); // Used with navX gyro

        // Update the pose
        odometry.update(
            gyroAngle,
            new SwerveModulePosition[] {
                    br_motor.getSwervePosition(),
                    fr_motor.getSwervePosition(),
                    fl_motor.getSwervePosition(),
                    bl_motor.getSwervePosition()
            }
        );
    }



    public void updateShuffleboard() {
        // SmartDashboard.putNumber("BR Position", br_motor.getSteeringPosition() - br_motor.getOffset());
        // SmartDashboard.putNumber("FR Position", fr_motor.getSteeringPosition() - fr_motor.getOffset());
        // SmartDashboard.putNumber("FL Position", fl_motor.getSteeringPosition() - fl_motor.getOffset());
        // SmartDashboard.putNumber("BL Position", bl_motor.getSteeringPosition() - bl_motor.getOffset());
        
        SmartDashboard.putNumber("BR Position", br_motor.getSteeringPosition());
        SmartDashboard.putNumber("FR Position", fr_motor.getSteeringPosition());
        SmartDashboard.putNumber("FL Position", fl_motor.getSteeringPosition());
        SmartDashboard.putNumber("BL Position", bl_motor.getSteeringPosition());

        SmartDashboard.putNumber("BR ABS Position", br_motor.getAbsoluteSteeringPosition());
        SmartDashboard.putNumber("FR ABS Position", fr_motor.getAbsoluteSteeringPosition());
        SmartDashboard.putNumber("FL ABS Position", fl_motor.getAbsoluteSteeringPosition());
        SmartDashboard.putNumber("BL ABS Position", bl_motor.getAbsoluteSteeringPosition());

        SmartDashboard.putNumber("BR Offset", br_motor.getOffset());
        SmartDashboard.putNumber("FR Offset", fr_motor.getOffset());
        SmartDashboard.putNumber("FL Offset", fl_motor.getOffset());
        SmartDashboard.putNumber("BL Offset", bl_motor.getOffset());

        SmartDashboard.putNumber("Odometry X", odometry.getPoseMeters().getX());
        SmartDashboard.putNumber("Odometry Y", odometry.getPoseMeters().getY());
        SmartDashboard.putNumber("Odometry Angle", odometry.getPoseMeters().getRotation().getDegrees());
    }

    // Motor functions
    public void move(double turnRatio, double inputTheta, double driveSpeed, double turnSpeed, boolean fieldRelative){
        double driveRatio = 1 - turnRatio; // Percent of total joystick movement dedicated to driving
        double theta = inputTheta;

        if (fieldRelative) {
            theta += m_gyro.getRotation2d().getRadians();
        }

        double br_angle = theta * driveRatio + 0.25 * turnRatio;
        double fr_angle = theta * driveRatio - 0.25 * turnRatio;
        double bl_angle = theta * driveRatio - 0.25 * turnRatio;
        double fl_angle = theta * driveRatio + 0.25 * turnRatio;

        double combinedSpeed = driveSpeed + turnSpeed;

        br_motor.steer(br_angle);
        fr_motor.steer(fr_angle);
        bl_motor.steer(bl_angle);
        fl_motor.steer(fl_angle);

        br_motor.drive(-combinedSpeed);
        fr_motor.drive(-combinedSpeed);
        bl_motor.drive(combinedSpeed);
        fl_motor.drive(combinedSpeed);
    }

    public void calibrateSteering(){
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

    public void pointStraight() {
        double goalAngle = 0;
        double currentAngle = (odometry.getPoseMeters().getRotation().getRadians()); // Module 2π?
        double turnSpeed = turningPIDController.calculate(currentAngle, goalAngle);

        turn(turnSpeed);
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

    public void turn(double r) {
        br_motor.steer(0.25);
        fr_motor.steer(-0.25);
        bl_motor.steer(-0.25);
        fl_motor.steer(0.25);

        br_motor.drive(r);
        fr_motor.drive(r);
        bl_motor.drive(r);
        fl_motor.drive(r);
    }
}
