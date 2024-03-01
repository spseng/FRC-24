package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import frc.robot.drive.Drivetrain;
import frc.robot.vision.VisionSystem;

public class AutonomousController {
    private StepManager stepManager;
    private Pose2d startPosition;

    public void init() {
        stepManager = new StepManager();
    }

    public void periodic(Drivetrain drivetrain, ShooterSystem shooterSystem, VisionSystem visionSystem) {
        if(startPosition == null){
            startPosition = drivetrain.getPose();
        }

        // Custom Class that will run these steps one at a time
        // Once the function returns true, it will move to the next step
        stepManager.runSteps(
                () -> shooterSystem.autonShoot(drivetrain.getPose()),
                () -> drivetrain.moveTo(startPosition.plus(new Transform2d(2, 0, new Rotation2d())))
//                 -> drivetrain.moveTo("amp"),
//                () -> shooterSystem.autonShootAmp(),
//                () -> drivetrain.moveTo(0, 0, 0)
        );
    }
}
