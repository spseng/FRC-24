package frc.robot;

// import com.kauailabs.navx.frc.*;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.*;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


import static frc.robot.Constants.*;

public class Drivetrain {

    // Locations for the swerve drive modules relative to the robot center (pretty sure it's based off CoM)
    private final Translation2d fl_location = new Translation2d( TRACKWIDTH/2,  WHEELBASE/2);
    private final Translation2d fr_location = new Translation2d( TRACKWIDTH/2, -WHEELBASE/2);
    private final Translation2d bl_location = new Translation2d(-TRACKWIDTH/2,  WHEELBASE/2);
    private final Translation2d br_location = new Translation2d(-TRACKWIDTH/2, -WHEELBASE/2);

    // private final Translation2d fl_location = new Translation2d( 0.03,  WHEELBASE/2);
    // private final Translation2d fr_location = new Translation2d( 0.03, -WHEELBASE/2);
    // private final Translation2d bl_location = new Translation2d(-TRACKWIDTH + 0.03,  WHEELBASE/2);
    // private final Translation2d br_location = new Translation2d(-TRACKWIDTH + 0.03, -WHEELBASE/2);

    // Creating my kinematics object using the module locations
    private final SwerveDriveKinematics driveKinematics = new SwerveDriveKinematics(
            fl_location, fr_location, bl_location, br_location
    );

     private final SwerveDriveOdometry odometry;

    private SwerveModuleState[] previousStates;

    private final SwerveMotor br_motor;
    private final SwerveMotor fr_motor;
    private final SwerveMotor fl_motor;
    private final SwerveMotor bl_motor;


    // Turning
    private final PIDController turningPIDController;

    private final AHRS gyro = new AHRS();  // navX gyro

    public Drivetrain() {
        fl_motor = new SwerveMotor(FL_STEER_CAN, FL_DRIVE_CAN, FL_STEER_OFFSET);
        fr_motor = new SwerveMotor(FR_STEER_CAN, FR_DRIVE_CAN, FR_STEER_OFFSET);
        bl_motor = new SwerveMotor(BL_STEER_CAN, BL_DRIVE_CAN, BL_STEER_OFFSET);
        br_motor = new SwerveMotor(BR_STEER_CAN, BR_DRIVE_CAN, BR_STEER_OFFSET);

         odometry = new SwerveDriveOdometry(
                 driveKinematics, gyro.getRotation2d(),
                 new SwerveModulePosition[] {
                        fl_motor.getSwervePosition(),
                        fr_motor.getSwervePosition(),
                        bl_motor.getSwervePosition(),
                        br_motor.getSwervePosition()
                 }, new Pose2d(0, 0, new Rotation2d()));

        this.turningPIDController = new PIDController(TURNING_KP, TURNING_KI, TURNING_KD);

        gyro.reset();
    }

    public void periodic() {
        // Get the rotation of the robot from the gyro.
        var gyroAngle = gyro.getRotation2d();

        // Update the pose
        odometry.update(gyroAngle,
            new SwerveModulePosition[] {
                fl_motor.getSwervePosition(), fr_motor.getSwervePosition(),
                bl_motor.getSwervePosition(), br_motor.getSwervePosition()
            });
    }



    public void updateShuffleboard() {
        SmartDashboard.putNumber("BR Position", br_motor.getSteeringPosition());
        SmartDashboard.putNumber("FR Position", fr_motor.getSteeringPosition());
        SmartDashboard.putNumber("FL Position", fl_motor.getSteeringPosition());
        SmartDashboard.putNumber("BL Position", bl_motor.getSteeringPosition());

    //    SmartDashboard.putNumber("BR ABS Position", br_motor.getAbsoluteSteeringPosition());
    //    SmartDashboard.putNumber("FR ABS Position", fr_motor.getAbsoluteSteeringPosition());
    //    SmartDashboard.putNumber("FL ABS Position", fl_motor.getAbsoluteSteeringPosition());
    //    SmartDashboard.putNumber("BL ABS Position", bl_motor.getAbsoluteSteeringPosition());

    //    SmartDashboard.putNumber("BR Offset", br_motor.getOffset());
    //    SmartDashboard.putNumber("FR Offset", fr_motor.getOffset());
    //    SmartDashboard.putNumber("FL Offset", fl_motor.getOffset());
    //    SmartDashboard.putNumber("BL Offset", bl_motor.getOffset());

         SmartDashboard.putNumber("Odometry X", odometry.getPoseMeters().getX());
         SmartDashboard.putNumber("Odometry Y", odometry.getPoseMeters().getY());
         SmartDashboard.putNumber("Odometry Angle", odometry.getPoseMeters().getRotation().getDegrees());
    }


    public void move(double driveX, double driveY, double heading) {
        ChassisSpeeds relativeSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(driveY, driveX, heading, gyro.getRotation2d());
        // ChassisSpeeds relativeSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(driveY, driveX, 0.5, gyro.getRotation2d());
        // ChassisSpeeds speeds = new ChassisSpeeds(driveY, driveX, 1);

        SwerveModuleState[] moduleStates = driveKinematics.toSwerveModuleStates(relativeSpeeds);

        previousStates = moduleStates;

        move(moduleStates);
    }

    private void move(SwerveModuleState[] moduleStates) {
        double fl_angle = moduleStates[0].angle.getRadians() / (2 * Math.PI) * FULL_ROTATION;
        double fr_angle = moduleStates[1].angle.getRadians() / (2 * Math.PI) * FULL_ROTATION;
        double bl_angle = moduleStates[2].angle.getRadians() / (2 * Math.PI) * FULL_ROTATION;
        double br_angle = moduleStates[3].angle.getRadians() / (2 * Math.PI) * FULL_ROTATION;

        double fl_speed = moduleStates[0].speedMetersPerSecond / MAX_SPEED;
        double fr_speed = moduleStates[1].speedMetersPerSecond / MAX_SPEED;
        double bl_speed = moduleStates[2].speedMetersPerSecond / MAX_SPEED;
        double br_speed = moduleStates[3].speedMetersPerSecond / MAX_SPEED;
        
        fl_motor.steer(fl_angle);
        fr_motor.steer(fr_angle);
        bl_motor.steer(bl_angle);
        br_motor.steer(br_angle);

        fl_motor.drive(fl_speed);
        fr_motor.drive(fr_speed);
        bl_motor.drive(bl_speed);
        br_motor.drive(br_speed);
    }

    public void moveTo(double x, double y, double heading) {
        double gyroAngle = gyro.getRotation2d().getDegrees() / 360 * FULL_ROTATION;
        double turningSpeed = turningPIDController.calculate(gyroAngle, heading) / FULL_ROTATION * 2 * Math.PI;

        Translation2d translation = new Translation2d(x - odometry.getPoseMeters().getX(), y - odometry.getPoseMeters().getY());
        SwerveModuleState[] moduleStates = driveKinematics.toSwerveModuleStates(
                ChassisSpeeds.fromFieldRelativeSpeeds(
                        translation.getX(), translation.getY(), turningSpeed, gyro.getRotation2d()
                )
        );

        move(moduleStates);
    }

    public void calibrateSteering(){
        // Zero the gyro
        gyro.zeroYaw();

        // Zero the odometry
        odometry.resetPosition(gyro.getRotation2d(), new SwerveModulePosition[]{
            fl_motor.getSwervePosition(),
            fr_motor.getSwervePosition(),
            bl_motor.getSwervePosition(),
            br_motor.getSwervePosition()
        }, new Pose2d());

        // Calibrate the relative encoders to match absolute encoders
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
        point(0);
    }

     public void point(double direction) {
        move(0, 0, direction);
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
        bl_motor.drive(-r);
        fl_motor.drive(-r);
    }


    public SwerveModuleState[] getGoalSwerveModuleStates(){
        return previousStates;
        
        // return new SwerveModuleState[] {
        //     fl_motor.getSwerveState(),
        //     fr_motor.getSwerveState(),
        //     bl_motor.getSwerveState(),
        //     br_motor.getSwerveState()
        // };
    }

    public SwerveModuleState[] getRealSwerveModuleStates(){        
        return new SwerveModuleState[] {
            fl_motor.getSwerveModuleState(),
            fr_motor.getSwerveModuleState(),
            bl_motor.getSwerveModuleState(),
            br_motor.getSwerveModuleState()
        };
    }
}
