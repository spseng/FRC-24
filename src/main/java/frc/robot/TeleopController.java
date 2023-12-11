package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import static frc.robot.Constants.*;

public class TeleopController {
    // Instance Variables

    public TeleopController() {

    }

    public void teleopInit(Drivetrain driveTrain) {
        // Calibrate relative encoders to match absolute encoders
        driveTrain.calibrate();
    }

    public void teleopPeriodic(XboxController m_stick, Drivetrain drivetrain) {
        double leftX = m_stick.getLeftX();
        double leftY = m_stick.getLeftY();
        double r = DRIVE_SPEED*Math.sqrt( Math.pow(leftX,2) + Math.pow(leftY,2) );
        double theta = getAngle(leftX,leftY);

        double rightX = m_stick.getRightX();
        double turnSpeed = TURN_SPEED * rightX;

        if (r > DRIVE_SPEED/2) {
            drivetrain.steer(theta);
            drivetrain.drive(r);
        } else
        if (Math.abs(rightX) > TURN_SPEED/2) {
            drivetrain.turn(turnSpeed, Math.signum(turnSpeed));
            drivetrain.drive(rightX);
        } else
        if (m_stick.getAButton()) {
            drivetrain.zeroSteering();
        } else
        if (m_stick.getYButton()) {
            drivetrain.steer(0);
        } else
        if (m_stick.getBButton()) {
            drivetrain.steer(1);
        } else
        if(m_stick.getAButtonReleased() || m_stick.getBButtonReleased()){
            drivetrain.stopSteering();
        } else {
            drivetrain.stopSteering();
            drivetrain.drive(0);
        }


        drivetrain.updateShuffleboard();
    }

    // Helper functions
    public double getAngle(double x, double y) {
        return (((Math.atan2(y, x))/Math.PI + 2.5) % 2);
    }
}
