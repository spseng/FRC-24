package frc.robot;

import edu.wpi.first.wpilibj.XboxController;

import static frc.robot.Constants.*;

public class TeleopController {
    // Instance Variables

    public TeleopController() {

    }

    public void teleopInit(Drivetrain driveTrain) {
        // Calibrate relative encoders to match absolute encoders
        driveTrain.calibrateSteering();
    }

    public void teleopPeriodic(XboxController m_stick, Drivetrain drivetrain) {
        double leftX = m_stick.getLeftX();
        double leftY = -m_stick.getLeftY();
        double theta = getAngle(leftX,leftY);

        double leftR = Math.sqrt( Math.pow(leftX,2) + Math.pow(leftY,2) );
        double driveSpeed = leftR * DRIVE_SPEED;

        double rightX = Math.abs(m_stick.getRightX()) < JOYSTICK_DEAD_ZONE ? 0 : m_stick.getRightX();
        double turnSpeed = TURN_SPEED * rightX;

        boolean doFieldOrientedDriving = (m_stick.getLeftTriggerAxis() > TRIGGER_DEAD_ZONE);

        if (leftR > JOYSTICK_DEAD_ZONE || Math.abs(rightX) > JOYSTICK_DEAD_ZONE) {
            double turnRatio = rightX / (rightX + leftR); // Percent of total joystick movement dedicated to turning
            drivetrain.move(turnRatio, theta, driveSpeed, turnSpeed, doFieldOrientedDriving);
        }else
        if (m_stick.getAButton()) {
            drivetrain.zeroSteering();
        } else
        if (m_stick.getYButton()) {
            drivetrain.steer(0);
        } else
        if (m_stick.getBButton()) {
            drivetrain.steer(1);
        } else
        if (m_stick.getXButton()) {
            drivetrain.pointStraight();
        } else
        if (m_stick.getRightBumperReleased()) {
            drivetrain.calibrateSteering();
        } else
        if(m_stick.getAButtonReleased() || m_stick.getBButtonReleased()){
            drivetrain.stopSteering();
        } else {
            drivetrain.stopSteering();
            drivetrain.drive(0);
        }

        drivetrain.periodic();

        drivetrain.updateShuffleboard();
    }

    // Helper functions
    public double getAngle(double x, double y) {
        return (((Math.atan2(y, -x))/Math.PI + 1*FULL_ROTATION) % FULL_ROTATION);
    }
}
