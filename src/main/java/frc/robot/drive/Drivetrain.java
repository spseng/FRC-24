package frc.robot.drive;

/// x import com.kauailabs.navx.frc.*;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.vision.FieldLayout;


import static frc.robot.Constants.*;

public class Drivetrain {

    private final Translation2d fl_location = new Translation2d(-TRACKWIDTH / 2, -WHEELBASE / 2);
    private final Translation2d fr_location = new Translation2d(-TRACKWIDTH / 2, WHEELBASE / 2);
    private final Translation2d bl_location = new Translation2d(TRACKWIDTH / 2, -WHEELBASE / 2);
    private final Translation2d br_location = new Translation2d(TRACKWIDTH / 2, WHEELBASE / 2);

    private final DoublePublisher turningGoal;
    private final DoublePublisher turningReal;

    // Creating my kinematics object using the module locations
    private final SwerveDriveKinematics driveKinematics = new SwerveDriveKinematics(
            fl_location, fr_location, bl_location, br_location
    );

    // private final SwerveDriveOdometry odometry;
    private final SwerveDrivePoseEstimator swervePoseEstimator;

    // private final PhotonCamera camera = new PhotonCamera("photonvision");

    private SwerveModuleState[] previousStates;

    private double goalHeading = 0;
    private double yawHeadingOffset = 0;

    private final SwerveMotor br_motor;
    private final SwerveMotor fr_motor;
    private final SwerveMotor fl_motor;
    private final SwerveMotor bl_motor;

    private double turnSpeed = 0;


    // Turning
    private final PIDController turningPIDController;
    private final PIDController driveXPIDController;
    private final PIDController driveYPIDController;

    private final AHRS gyro = new AHRS();  // navX gyro

    public Drivetrain() {
        fl_motor = new SwerveMotor(FL_STEER_CAN, FL_DRIVE_CAN, FL_STEER_OFFSET);
        fr_motor = new SwerveMotor(FR_STEER_CAN, FR_DRIVE_CAN, FR_STEER_OFFSET);
        bl_motor = new SwerveMotor(BL_STEER_CAN, BL_DRIVE_CAN, BL_STEER_OFFSET);
        br_motor = new SwerveMotor(BR_STEER_CAN, BR_DRIVE_CAN, BR_STEER_OFFSET);

        gyro.reset();

        // odometry = new SwerveDriveOdometry(
        //          driveKinematics,
        //         gyro.getRotation2d(),
        //          new SwerveModulePosition[] {
        //                 fl_motor.getSwervePosition(),
        //                 fr_motor.getSwervePosition(),
        //                 bl_motor.getSwervePosition(),
        //                 br_motor.getSwervePosition()
        //          }, new Pose2d(0, 0, new Rotation2d(Math.PI)));

        swervePoseEstimator = new SwerveDrivePoseEstimator(
                driveKinematics,
                gyro.getRotation2d(),
                new SwerveModulePosition[]{
                        fl_motor.getSwervePosition(),
                        fr_motor.getSwervePosition(),
                        bl_motor.getSwervePosition(),
                        br_motor.getSwervePosition()
                }, new Pose2d(0, 0, new Rotation2d(Math.PI)));

        this.turningPIDController = new PIDController(TURNING_KP, TURNING_KI, TURNING_KD);

        this.driveXPIDController = new PIDController(DRIVE_KP, DRIVE_KI, DRIVE_KD);
        this.driveYPIDController = new PIDController(DRIVE_KP, DRIVE_KI, DRIVE_KD);

        calibrateSteering();

        turningReal = NetworkTableInstance.getDefault().getDoubleTopic("/turningReal").publish();
        turningGoal = NetworkTableInstance.getDefault().getDoubleTopic("/turningGoal").publish();
    }

    public void periodic() {
        // Get the rotation of the robot from the gyro.
        var gyroAngle = gyro.getRotation2d();

        // Update the pose
        swervePoseEstimator.update(gyroAngle,
                new SwerveModulePosition[]{
                        fl_motor.getSwervePosition(), fr_motor.getSwervePosition(),
                        bl_motor.getSwervePosition(), br_motor.getSwervePosition()
                });



        updateShuffleboard();
    }

    public void updateShuffleboard() {
        SmartDashboard.putNumber("FL Position", fl_motor.getSteeringPosition());
        SmartDashboard.putNumber("FR Position", fr_motor.getSteeringPosition());
        SmartDashboard.putNumber("BL Position", bl_motor.getSteeringPosition());
        SmartDashboard.putNumber("BR Position", br_motor.getSteeringPosition());

        SmartDashboard.putNumber("Odometry X", getPose().getX());
        SmartDashboard.putNumber("Odometry Y", getPose().getY());
        SmartDashboard.putNumber("Odometry Angle", getPose().getRotation().getDegrees());
    }

    private double closestAngle(double previous, double goal) {
        // // get direction
        // double dir = nearestRotation(goal) - nearestRotation(previous);

        // // If rotation is greater than 180 degrees, then rotate swerve in the other way
        // if (Math.abs(dir) > FULL_ROTATION / 2) {
        //     dir = -(Math.signum(dir) * FULL_ROTATION) + dir;
        // }

        return goal;
    }

    // For some reason, the built-in modulo function didn't work...
    private static double nearestRotation(double angle) {
        return angle - FULL_ROTATION * Math.floor(angle / FULL_ROTATION);
    }

    public void setYawHeadingOffset(double offset) {
        yawHeadingOffset = offset;
    }

    public void pointTowards(Pose2d goal) {
        Translation2d translation = goal.getTranslation().minus(getPose().getTranslation());
//        Translation2d translation = new Translation2d(goal.getTranslation().getX() - odometry.getPoseMeters().getX(), goal.getTranslation().getY() - odometry.getPoseMeters().getY());
        double angle = Math.atan2(translation.getY(), translation.getX()); // This is between -pi and pi
        double adjustedAngle = angle / (2 * Math.PI) * FULL_ROTATION;
        setGoalHeading(adjustedAngle);
    }

    public void rotate(double byAmount) {
        turnSpeed = byAmount * TURN_SPEED;
    }

    public void move(double driveX, double driveY, double heading) {
        setGoalHeading(heading);
        move(driveX, driveY);
    }


    public void move(){
        move(0, 0);
    }

    public void move(double driveX, double driveY) {
        double adjustedGyroAngle = gyro.getRotation2d().getDegrees() / 360.0 * FULL_ROTATION;

        double closestAngle = closestAngle(adjustedGyroAngle, getGoalHeading());

        double turningSpeed = turningPIDController.calculate(closestAngle, 0);
        turningSpeed = Math.abs(turningSpeed) > MIN_TURNING_SPEED ? Math.min(MAX_TURING_SPEED, Math.max(-MAX_TURING_SPEED, turningSpeed)) : 0;

        turningReal.set(closestAngle);
        turningGoal.set(0);

        ChassisSpeeds relativeSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(driveY, driveX, turnSpeed / FULL_ROTATION * Math.PI * 2, gyro.getRotation2d());

        SwerveModuleState[] moduleStates = driveKinematics.toSwerveModuleStates(relativeSpeeds);

        move(moduleStates);
    }

    private void move(SwerveModuleState[] moduleStates) {

        previousStates = moduleStates;

//        var frontLeftOptimized = SwerveModuleState.optimize(moduleStates[0], Rotation2d.fromDegrees(convertSwerveStateRange(fl_motor.getSteeringPosition())));
//        var frontRightOptimized = SwerveModuleState.optimize(moduleStates[1], Rotation2d.fromDegrees(convertSwerveStateRange(fr_motor.getSteeringPosition())));
//        var backLeftOptimized = SwerveModuleState.optimize(moduleStates[2], Rotation2d.fromDegrees(convertSwerveStateRange(bl_motor.getSteeringPosition())));
//        var backRightOptimized = SwerveModuleState.optimize(moduleStates[3], Rotation2d.fromDegrees(convertSwerveStateRange(br_motor.getSteeringPosition())));

        fl_motor.setModuleState(moduleStates[0]);
        fr_motor.setModuleState(moduleStates[1]);
        bl_motor.setModuleState(moduleStates[2]);
        br_motor.setModuleState(moduleStates[3]);


//        fl_motor.setModuleState(frontLeftOptimized);
//        fr_motor.setModuleState(frontRightOptimized);
//        bl_motor.setModuleState(backLeftOptimized);
//        br_motor.setModuleState(backRightOptimized);

        // fl_motor.drive(fl_speed);
        // fr_motor.drive(fr_speed);
        // bl_motor.drive(bl_speed);
        // br_motor.drive(br_speed);
    }

    private double convertSwerveStateRange(double rotations) {
        return (rotations/FULL_ROTATION - 0.5) * 360;
    }

    public boolean moveTo(String nearestLocationCalled){
        return switch (nearestLocationCalled) {
            case "amp" -> moveTo(getPose().nearest(FieldLayout.ampLocations));
            case "shoot" -> moveTo(getPose().nearest(FieldLayout.shootingLocations));
//            case "collection" -> moveTo(3, 2, 90); // This requires us to know which of the three slots to go to.
            default -> false;
        };
    }

    public boolean moveTo(Pose2d goal) {
        return moveTo(goal.getX(), goal.getY(), goal.getRotation().getDegrees());
    }

    public boolean moveTo(double x, double y, double angle) {
        double endHeading = angle / 360 * FULL_ROTATION;
        setGoalHeading(endHeading);

        double gyroAngle = gyro.getRotation2d().getDegrees() / 360 * FULL_ROTATION;
        double turningSpeed = turningPIDController.calculate(gyroAngle, getGoalHeading()) / FULL_ROTATION * 2 * Math.PI;

        Pose2d currentPose = getPose();
        Translation2d translation = new Translation2d(x - currentPose.getX(), y - currentPose.getY());

        // Drive PID Controller towards endpoint
        double driveXSpeed = driveXPIDController.calculate(translation.getX(), 0);
        double driveYSpeed = driveYPIDController.calculate(translation.getY(), 0);

        ChassisSpeeds relativeSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(driveYSpeed, driveXSpeed, turningSpeed, gyro.getRotation2d());
        SwerveModuleState[] moduleStates = driveKinematics.toSwerveModuleStates(relativeSpeeds);
        move(moduleStates);

        return translation.getNorm() < AUTONOMOUS_POSITION_MAX_ERROR && Math.abs(gyroAngle - getGoalHeading()) < AUTONOMOUS_POSITION_MAX_ERROR;
    }

    public void calibrateSteering() {
        // Zero the odometry
        swervePoseEstimator.resetPosition(
                Rotation2d.fromDegrees(0),
                new SwerveModulePosition[]{
                        zeroPosition(),
                        zeroPosition(),
                        zeroPosition(),
                        zeroPosition()
                }, new Pose2d(0, 0, Rotation2d.fromDegrees(0))
        );

        // Calibrate the relative encoders to match absolute encoders
        br_motor.calibrate();
        fr_motor.calibrate();
        bl_motor.calibrate();
        fl_motor.calibrate();

        // Zero the gyro
        gyro.zeroYaw();

        setGoalHeading(0);
        setYawHeadingOffset(0);
    }

    public SwerveModulePosition zeroPosition() {
        return new SwerveModulePosition(0, Rotation2d.fromDegrees(0));
    }

    public void pointStraight() {
        setGoalHeading(0);
        move(0, 0);
    }

    public SwerveDrivePoseEstimator getPoseEstimator() {
        return swervePoseEstimator;
    }

    public SwerveModuleState[] getGoalSwerveModuleStates() {
        return previousStates;
    }

    public SwerveModuleState[] getRealSwerveModuleStates() {
        return new SwerveModuleState[]{
                fl_motor.getSwerveModuleState(),
                fr_motor.getSwerveModuleState(),
                bl_motor.getSwerveModuleState(),
                br_motor.getSwerveModuleState()
        };
    }

    public Pose2d getPose() {
        // return new Pose2d(3, 8, new Rotation2d());
        return swervePoseEstimator.getEstimatedPosition();
    }

    public double getGoalHeading() {
        return goalHeading + yawHeadingOffset;
    }

    public void setGoalHeading(double newHeading) {
        goalHeading = newHeading;
    }
}
