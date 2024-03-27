package frc.robot.Commands;

import frc.robot.Constants;
import frc.robot.ShooterSystem;

public class SetArmAngleAmp extends BigRedCommand {
    private ShooterSystem _shooter;
    public SetArmAngleAmp(ShooterSystem shooter){
        _shooter = shooter;

    }
    public void excecute(){
        _shooter.setArmAngle(Constants.AMP_SCORING_ANGLE);
    }
    public boolean isFinished(){
        return (_shooter.getAngle() >= (Constants.AMP_SCORING_ANGLE - .05));
    }
}
