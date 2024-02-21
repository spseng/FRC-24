package frc.robot.vision;

import edu.wpi.first.math.geometry.Pose3d;
import frc.robot.Drivetrain;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonTrackedTarget;


public class VisionController {
    private double yaw;

    // TODO: Make the cameras an arraylist of RobotCamera
    private RobotCamera camera = new RobotCamera("Camera", 0, 0, 0, 0, 0);

    private Pose3d measuredPose;

    public void periodic() {
        // This method will be called once per scheduler run
        var photonVisionLatestResult = camera.getLatestResult();
        SmartDashboard.putBoolean("Camera has Target", photonVisionLatestResult.hasTargets());
    
        if (photonVisionLatestResult.hasTargets()){
            var target = photonVisionLatestResult.getBestTarget();
            updateYaw(target);

            measuredPose = PhotonUtils.estimateFieldToRobotAprilTag(target.getBestCameraToTarget(), AprilTagFieldLayout.getTagPose(target.getFiducialId()), camera.getRelativeTransform());
        }

        updateShuffleboard();
    }

    public void updateDrivetrainPosition(Drivetrain drivetrain) {

        var res = camera.getLatestResult();
        if (res.hasTargets()) {
            var imageCaptureTime = res.getTimestampSeconds();
            var bestTarget = res.getBestTarget();

            // TODO: Maybe use bestTarget.getPoseAmbiguity() to know when to use the vision or skip it

            var camToTargetTrans = bestTarget.getBestCameraToTarget();
            Pose3d targetPosition = AprilTagFieldLayout.getTagPose(bestTarget.getFiducialId());
            var camPose = targetPosition.transformBy(camToTargetTrans.inverse());
            drivetrain.getPoseEstimator().addVisionMeasurement(
                    camPose.transformBy(camera.getRelativeTransform()).toPose2d(), imageCaptureTime);
        }
    }

    public void updateShuffleboard() {
        SmartDashboard.putNumber("Yaw", yaw);


        if(measuredPose != null){
            SmartDashboard.putNumber("Measured X", measuredPose.getTranslation().getX());
            SmartDashboard.putNumber("Measured Y", measuredPose.getTranslation().getY());
            SmartDashboard.putNumber("Measured Z", measuredPose.getTranslation().getZ());
            SmartDashboard.putNumber("Measured Pitch", measuredPose.getRotation().getAngle());
        }
    }

    // PhotonVisionResult may need to be changed
    public void updateYaw(PhotonTrackedTarget target) {
        yaw = target.getYaw();
    }


    // Getters and Setters
    public double getTargetRelativeYaw() {
        return yaw;
    }

    public Pose3d getMeasuredPose() {
        return measuredPose;
    }
}
