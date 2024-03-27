package frc.robot.Commands;

import frc.robot.Constants;
import frc.robot.ShooterSystem;

public class SetArmAngleIntake extends BigRedCommand {
    private ShooterSystem _shooter;
    public SetArmAngleIntake(ShooterSystem shooter){
        _shooter = shooter;
    }
    public void execute(){
        _shooter.setArmAngle(Constants.ARM_INTAKE_ANGLE);
    }
    public boolean isFinished(){
        return (_shooter.getAngle() >= (Constants.ARM_INTAKE_ANGLE + .05));
    }
    
}
