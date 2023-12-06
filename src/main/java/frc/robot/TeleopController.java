package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import static frc.robot.Constants.*;

public class TeleopController {
    // Instance Variables
    private final SwerveMotor fl_motor;
    private final SwerveMotor fr_motor;
    private final SwerveMotor bl_motor;
    private final SwerveMotor br_motor;

    public TeleopController() {
        fl_motor = new SwerveMotor(FL_STEER_CAN, FL_DRIVE_CAN, FL_STEER_OFFSET);
        fr_motor = new SwerveMotor(FR_STEER_CAN, FR_DRIVE_CAN, FR_STEER_OFFSET);
        bl_motor = new SwerveMotor(BL_STEER_CAN, BL_DRIVE_CAN, BL_STEER_OFFSET);
        br_motor = new SwerveMotor(BR_STEER_CAN, BR_DRIVE_CAN, BR_STEER_OFFSET);
    }
    public void teleopPeriodic(XboxController m_stick) {
        double x = - m_stick.getLeftX();
        double y = m_stick.getLeftY();
        double r = DRIVE_SPEED*Math.sqrt( Math.pow(x,2) + Math.pow(y,2) );
        double theta = getAngle(x,y);

        if (r > DRIVE_SPEED/2) {
            steer(theta);
            drive(r);
        }else 
        if (m_stick.getAButton()) {
            zeroSteering();
        } else
        if (m_stick.getYButton()) {
            steer(0);
        } else
        if (m_stick.getBButton()) {
            steer(1);
        } else
        if(m_stick.getAButtonReleased() || m_stick.getBButtonReleased()){
            stopSteering();
        } else {
            stopSteering();
            drive(0);
        }

        updateShuffleboard();
    }

    private void updateShuffleboard() {
        SmartDashboard.putNumber("BR Position", br_motor.getSteeringPosition());
        SmartDashboard.putNumber("FR Position", fr_motor.getSteeringPosition());
        SmartDashboard.putNumber("FL Position", fl_motor.getSteeringPosition());


        SmartDashboard.putNumber("BL Position", bl_motor.getSteeringPosition());
    }

    // Motor functions
    private void zeroSteering() {
        br_motor.zeroPosition();
        fr_motor.zeroPosition();
        bl_motor.zeroPosition();
        fl_motor.zeroPosition();
    }

    private void stopSteering() {
        br_motor.stopSteering();
        fr_motor.stopSteering();
        bl_motor.stopSteering();
        fl_motor.stopSteering();
    }

    private void steer(double theta) {
        br_motor.steer(theta);
        fr_motor.steer(theta);
        bl_motor.steer(theta);
        fl_motor.steer(theta);
    }

    private void drive(double r) {
        br_motor.drive(r);
        fr_motor.drive(r);
        bl_motor.drive(r);
        fl_motor.drive(r);
    }

    // Helper functions
    public double getAngle(double x, double y) {
        return (((Math.atan2(y, x))/Math.PI + 2.5) % 2);
    }
}
