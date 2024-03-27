package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import frc.robot.drive.Drivetrain;
import frc.robot.vision.VisionSystem;

public class AutonomousController {
    private StepManager stepManager;
    private Pose2d startPosition;

    double counter = 0;

    public void init() {
        stepManager = new StepManager();
        counter = 0;
    }

    public void periodic(Drivetrain drivetrain, ShooterSystem shooterSystem) {
        if(startPosition == null){
            startPosition = drivetrain.getPose();
        }

        counter++;
        if(counter < 110){
            drivetrain.move(0,0.5);
        }else {
            drivetrain.move(0,0);
        }
//         // Custom Class that will run these steps one at a time
//         // Once the function returns true, it will move to the next step
        // stepManager.runSteps(
// //                 // () -> shooterSystem.autonShoot(drivetrain.getPose()),
//                 () -> drivetrain.moveTo(startPosition.plus(new Transform2d(2, 0, new Rotation2d()))) // BLUE
                // () -> drivetrain.moveTo(startPosition.plus(new Transform2d(-2, 0, new Rotation2d()))) // RED
// //                 -> drivetrain.moveTo("amp"),
// //                () -> shooterSystem.autonShootAmp(),
// //                () -> drivetrain.=moveTo(0, 0, 0)
        // );
        

        
    }
}
