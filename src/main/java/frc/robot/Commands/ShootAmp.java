package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.ShooterSystem;

public class ShootAmp extends BigRedCommand {
    private ShooterSystem _shooter;
    public ShootAmp(ShooterSystem shooter){
        _shooter = shooter;


        
    }
    public void excecute(){
        new SequentialCommandGroup(new SetArmAngleAmp(_shooter), new Shoot(_shooter));

        
        

    }
}
