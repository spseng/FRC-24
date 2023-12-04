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


        public double getAngle(double x, double y) {
        double rad = Math.sqrt(Math.pow(x, 2)+Math.pow(y, 2));
        double theta=0;
        if(x>=0 ){
            rad = rad;
            // quad 1
            theta = Math.atan2(y,x);
        } else if (x<0 ) {
            rad = -rad;
            theta = Math.atan2(y,-x) + Math.PI;
        }
        return theta;
    }

    public TeleopController() {
        fl_motor = new SwerveMotor(FL_STEER_CAN, FL_DRIVE_CAN, FL_STEER_OFFSET, true);
        fr_motor = new SwerveMotor(FR_STEER_CAN, FR_DRIVE_CAN, FR_STEER_OFFSET, false);
        bl_motor = new SwerveMotor(BL_STEER_CAN, BL_DRIVE_CAN, BL_STEER_OFFSET, true);
        br_motor = new SwerveMotor(BR_STEER_CAN, BR_DRIVE_CAN, BR_STEER_OFFSET, false);
    }
    double prevTheta = 0;
    public void teleopPeriodic(XboxController m_stick) {
        double x = m_stick.getLeftX();
        double y = m_stick.getLeftY();
        double r = DRIVE_SPEED*Math.sqrt( Math.pow(x,2) + Math.pow(y,2) );
        double theta = getAngle(x,y)/Math.PI-.5;
        if (x>0){
            // r=-r;
            // theta=1-theta;
        }
        // Account for joystick deadzone
        if (Math.abs(r) > 0.1) {
            // TODO: Conditional checking for various forms of turning

            br_motor.steer(theta);
            fr_motor.steer(theta);
            bl_motor.steer(theta);
            fl_motor.steer(theta);
            SmartDashboard.putNumber("Theta", theta);

            
            prevTheta=theta;
            br_motor.drive(r);
            fr_motor.drive(r);
            bl_motor.drive(r);
            fl_motor.drive(r);
            
        } else{
            br_motor.steer(prevTheta);
            fr_motor.steer(prevTheta);
            bl_motor.steer(prevTheta);
            fl_motor.steer(prevTheta);
            br_motor.drive(0);
            fr_motor.drive(0);
            bl_motor.drive(0);
            fl_motor.drive(0);

        }
        if (m_stick.getAButton()) {
            br_motor.zeroPosition();
            fr_motor.zeroPosition();
            bl_motor.zeroPosition();
            fl_motor.zeroPosition();
        }
        if (m_stick.getYButton()) {
            br_motor.steer(0);
            fr_motor.steer(0);
            bl_motor.steer(0);
            fl_motor.steer(0);
        }
        if (m_stick.getBButton()) {
            br_motor.steer(1);
            fr_motor.steer(1);
            bl_motor.steer(1);
            fl_motor.steer(1);
        }
        if(m_stick.getAButtonReleased() || m_stick.getBButtonReleased()){
            br_motor.stopSteering();
        }

        updateShuffleboard();
    }


    private void updateShuffleboard() {
        SmartDashboard.putNumber("BR Position", br_motor.getSteeringPosition());
        SmartDashboard.putNumber("FR Position", fr_motor.getSteeringPosition());
        SmartDashboard.putNumber("FL Position", fl_motor.getSteeringPosition());
        SmartDashboard.putNumber("BL Position", bl_motor.getSteeringPosition());
        // SmartDashboard.putNumber("Theta", prevAngle);
        // SmartDashboard.putNumber("R", r);
        // SmartDashboard.putNumber("X", x);
        // SmartDashboard.putNumber("Y", y);
    }
}
