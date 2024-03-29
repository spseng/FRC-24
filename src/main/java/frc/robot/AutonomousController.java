package frc.robot;

import static frc.robot.Constants.SPEAKER_SCORING_ANGLE;

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
        shootAuton(counter, drivetrain, shooterSystem);
        // backupAuton(counter, drivetrain);
       
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

    private void backupAuton(double counter, Drivetrain drivetrain) {
        if (counter < 35) {

        drivetrain.move(0,0.5);
        } else {
            drivetrain.move(0,0);
        }
    }

    private void shootAuton(double counter, Drivetrain drivetrain, ShooterSystem shooterSystem) {
         if(counter < 10) {
            shooterSystem.setArmRotation(SPEAKER_SCORING_ANGLE);
        } else if (counter < 160) {
            shooterSystem.shootMaxSpeed();
        } else if(counter < 560 && counter > 500){
            drivetrain.move(0,0.5);
        } else {
            drivetrain.move(0,0);
        }
    }
}
