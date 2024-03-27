package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.ShooterSystem;

public class Shoot extends BigRedCommand {
    
private ShooterSystem _shooter;
public Shoot(ShooterSystem shooterSystem){
    _shooter = shooterSystem;
    



}
public void execute(){
 
    _shooter.shootMaxSpeed();
}

}







