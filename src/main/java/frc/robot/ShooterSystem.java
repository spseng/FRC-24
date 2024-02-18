package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ShooterSystem {


    
    // CANVenom intakeMotor = new CANVenom()
    // private final Spark intakeMotor;
    // private final Spark shooterMotor;

    private final CANSparkMax intakeMotor;
    private final CANSparkMax shooterMotor;

    private double shootEndDelay = 2.0;
    private double shootDelay = 1.5;
    private double shootDelayCounter = 0;

    private final DigitalInput isLoadedButton;

    public ShooterSystem(int intakeMotorCAN, int shooterMotorCAN, int isLoadedButtonChannel){
        // intakeMotor = new Spark(intakeMotorChannel);
        // shooterMotor = new Spark(shooterMotorChannel);
        intakeMotor = new CANSparkMax(intakeMotorCAN, MotorType.kBrushless);
        shooterMotor = new CANSparkMax(shooterMotorCAN, MotorType.kBrushless);

        isLoadedButton = new DigitalInput(isLoadedButtonChannel);

    }

    public void updateShuffleboard() {
        SmartDashboard.putBoolean("IntakeLoaded", isLoaded());
    }

    public void intakeUnlessLoaded(){
        if(!isLoaded()){
            intakeMotor.set(0.3);
            shooterMotor.set(-0.05);
        } else {
            intakeMotor.stopMotor();
            shooterMotor.stopMotor();
        }
    }

    public void shoot(){
        shooterMotor.set(1.0);
        shootDelayCounter = 0.0;
    }

    public void periodic(double dt) {
        if(shootDelayCounter < shootDelay) {
            shootDelayCounter += dt;    
            shooterMotor.set(1.0);
        } else if (shootDelayCounter < shootEndDelay) {
            shootDelayCounter += dt;
            intakeMotor.set(0.5);
        }else {
            intakeMotor.stopMotor();
            shooterMotor.stopMotor();
        }

        updateShuffleboard();
    }


    public boolean isLoaded(){
        return !isLoadedButton.get();
    }
}
