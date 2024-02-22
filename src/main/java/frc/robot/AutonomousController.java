package frc.robot;

import frc.robot.drive.Drivetrain;
import frc.robot.vision.VisionSystem;

public class AutonomousController {
    private StepManager stepManager;

    public void init() {
        stepManager = new StepManager();
    }

    public void periodic(Drivetrain drivetrain, ShooterSystem shooterSystem, VisionSystem visionSystem) {
        // Custom Class that will run these steps one at a time
        // Once the function returns true, it will move to the next step
        stepManager.runSteps(
                () -> drivetrain.moveTo(2, 2, 0),
                () -> drivetrain.moveTo("amp"),
                () -> shooterSystem.autonShootAmp(),
                () -> drivetrain.moveTo(0, 0, 0)
        );
    }
}
