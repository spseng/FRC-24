package frc.robot;

// import com.kauailabs.navx.frc.*;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


import static frc.robot.Constants.*;

public class Drivetrain {
    
    private final Translation2d fl_location = new Translation2d( -TRACKWIDTH/2,  -WHEELBASE/2);
    private final Translation2d fr_location = new Translation2d( -TRACKWIDTH/2, WHEELBASE/2);
    private final Translation2d bl_location = new Translation2d(TRACKWIDTH/2,  -WHEELBASE/2);
    private final Translation2d br_location = new Translation2d(TRACKWIDTH/2, WHEELBASE/2);

    private final DoublePublisher turningGoal;
    private final DoublePublisher turningReal;

    // Creating my kinematics object using the module locations
    private final SwerveDriveKinematics driveKinematics = new SwerveDriveKinematics(
            fl_location, fr_location, bl_location, br_location
    );

    private final SwerveDriveOdometry odometry;

    // private final PhotonCamera camera = new PhotonCamera("photonvision");

    private SwerveModuleState[] previousStates;

    private double goalHeading = 0;

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

        gyro.reset();

        odometry = new SwerveDriveOdometry(
                 driveKinematics,
                gyro.getRotation2d(),
                 new SwerveModulePosition[] {
                        fl_motor.getSwervePosition(),
                        fr_motor.getSwervePosition(),
                        bl_motor.getSwervePosition(),
                        br_motor.getSwervePosition()
                 }, new Pose2d(0, 0, new Rotation2d(Math.PI)));

        this.turningPIDController = new PIDController(TURNING_KP, TURNING_KI, TURNING_KD);

        calibrateSteering();

        turningReal = NetworkTableInstance.getDefault().getDoubleTopic("/turningReal").publish();
        turningGoal = NetworkTableInstance.getDefault().getDoubleTopic("/turningGoal").publish();
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
        SmartDashboard.putNumber("FL Position", fl_motor.getSteeringPosition());
        SmartDashboard.putNumber("FR Position", fr_motor.getSteeringPosition());
        SmartDashboard.putNumber("BL Position", bl_motor.getSteeringPosition());
        SmartDashboard.putNumber("BR Position", br_motor.getSteeringPosition());

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

    private double closestAngle(double previous, double goal)
    {
        // get direction
        double dir = nearestRotation(goal) - nearestRotation(previous);
        
        // If rotation is greater than 180 degrees, then rotate swerve in the other way
        if (Math.abs(dir) > FULL_ROTATION/2)
        {
            dir = -(Math.signum(dir) * FULL_ROTATION) + dir;
        }

        return dir;
    }

    // For some reason the built-in modulo function didn't work...
    private static double nearestRotation(double angle) {
        return angle - FULL_ROTATION * Math.floor(angle / FULL_ROTATION);
    }

    public void rotate(double byAmount) {
        goalHeading += byAmount * TURN_SPEED;
    }

    public void move(double driveX, double driveY, double heading) {
        setHeading(heading);
        move(driveX, driveY);
    }

    public void move(double driveX, double driveY) {
        double adjustedGyroAngle = gyro.getRotation2d().getDegrees() / 360.0 * FULL_ROTATION;

        double closestAngle = closestAngle(adjustedGyroAngle, goalHeading);

        // TODO: Make this PID
        double heading = turningPIDController.calculate(closestAngle, 0);
        heading = Math.abs(heading) > MIN_TURNING_SPEED ? Math.min(MAX_TURING_SPEED, Math.max(-MAX_TURING_SPEED, heading)) : 0;

        turningReal.set(closestAngle);
        turningGoal.set(0);

        ChassisSpeeds relativeSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(driveY, driveX, heading / FULL_ROTATION * Math.PI * 2, gyro.getRotation2d());
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

        // double fl_speed = Math.max(moduleStates[0].speedMetersPerSecond * DRIVE_SPEED, -MAX_DRIVE_SPEED);
        double fl_speed = moduleStates[0].speedMetersPerSecond * DRIVE_SPEED;
        double fr_speed = moduleStates[1].speedMetersPerSecond * DRIVE_SPEED;
        double bl_speed = moduleStates[2].speedMetersPerSecond * DRIVE_SPEED;
        double br_speed = moduleStates[3].speedMetersPerSecond * DRIVE_SPEED;
        
        fl_motor.steer(fl_angle);
        fr_motor.steer(fr_angle);
        bl_motor.steer(bl_angle);
        br_motor.steer(br_angle);

        fl_motor.drive(fl_speed);
        fr_motor.drive(fr_speed);
        bl_motor.drive(bl_speed);
        br_motor.drive(br_speed);
    }

    public void moveTo(double x, double y, double endHeading) { // TODO: Make this move using PID
        setHeading(endHeading);
        double gyroAngle = gyro.getRotation2d().getDegrees() / 360 * FULL_ROTATION;
        double turningSpeed = turningPIDController.calculate(gyroAngle, goalHeading) / FULL_ROTATION * 2 * Math.PI;

        Translation2d translation = new Translation2d(x - odometry.getPoseMeters().getX(), y - odometry.getPoseMeters().getY());
        SwerveModuleState[] moduleStates = driveKinematics.toSwerveModuleStates(
                ChassisSpeeds.fromFieldRelativeSpeeds(
                        translation.getX(), translation.getY(), turningSpeed, gyro.getRotation2d()
                )
        );

        move(moduleStates);
    }


    /*
    public void faceNearestAprilTag(){
        // Vision-alignment mode
        // Query the latest result from PhotonVision
        var result = camera.getLatestResult();
        double rotationSpeed = 0;

        if (result.hasTargets()) {
            // Calculate angular turn power
            // -1.0 required to ensure positive PID controller effort _increases_ yaw
            rotationSpeed = -turningPIDController.calculate(result.getBestTarget().getYaw(), 0);
        } else {
            // If we have no targets, stay still.
            rotationSpeed = 0;
        }        
    }
    */

    public void calibrateSteering(){
        // Zero the odometry
        odometry.resetPosition(
            Rotation2d.fromDegrees(0),
         new SwerveModulePosition[]{
            zeroPosition(),
            zeroPosition(),
            zeroPosition(),
            zeroPosition()
        }, new Pose2d(0, 0, Rotation2d.fromDegrees(0)));

        // // Zero the odometry
        // odometry.resetPosition(
        //     gyro.getRotation2d(),
        //  new SwerveModulePosition[]{
        //     fl_motor.getSwervePosition(),
        //     fr_motor.getSwervePosition(),
        //     bl_motor.getSwervePosition(),
        //     br_motor.getSwervePosition()
        // }, new Pose2d(1, 1, Rotation2d.fromDegrees(0)));

     // Calibrate the relative encoders to match absolute encoders
        br_motor.calibrate();
        fr_motor.calibrate();
        bl_motor.calibrate();
        fl_motor.calibrate();

        // Zero the gyro
        gyro.zeroYaw();
    }

    public SwerveModulePosition zeroPosition() {
        return new SwerveModulePosition(0, Rotation2d.fromDegrees(0));
    }

    public void zeroSteering() {
        br_motor.zeroPosition();
        fr_motor.zeroPosition();
        bl_motor.zeroPosition();
        fl_motor.zeroPosition();
    }

    public void pointStraight() {
        setHeading(0);
        move(0, 0);
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

    public Pose2d getPose() {
        return odometry.getPoseMeters();
    }

    public void setHeading(double newHeading) {
        goalHeading = newHeading;
    }
}
