package frc.robot;

import edu.wpi.first.wpilibj.XboxController;

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
        double x = m_stick.getLeftX();
        double y = m_stick.getLeftY();
        double r = DRIVE_SPEED*Math.sqrt( Math.pow(x,2) + Math.pow(y,2) );
        double theta = Math.atan2(y,-x);

        // Account for joystick deadzone
        if (r > DRIVE_SPEED/2) {
            // TODO: Conditional checking for various forms of turning

            br_motor.steer(theta);
            fr_motor.steer(theta);
            bl_motor.steer(theta);
            fl_motor.steer(theta);
            br_motor.drive(r);
            fr_motor.drive(r);
            bl_motor.drive(r);
            fl_motor.drive(r);
        }

        if (m_stick.getAButton()) {
            br_motor.zeroPosition();
            fr_motor.zeroPosition();
            bl_motor.zeroPosition();
            fl_motor.zeroPosition();
        }
    }
}
