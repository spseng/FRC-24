package frc.robot.Commands;

import frc.robot.Constants;
import frc.robot.ShooterSystem;

public class SetArmAngleIntake extends BigRedCommand {
    private ShooterSystem _shooter;
    public SetArmAngleIntake(ShooterSystem shooter){
        _shooter = shooter;
    }
    public void execute(){
        _shooter.setArmRotation(Constants.ARM_INTAKE_ANGLE);
    }
    public boolean isFinished(){
        return (_shooter.getEncoderPosition() >= (Constants.ARM_INTAKE_ANGLE + .05));
    }
    
}
