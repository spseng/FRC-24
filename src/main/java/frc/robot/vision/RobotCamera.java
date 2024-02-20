package frc.robot.vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;

public class RobotCamera {
    private String name;
    private double relativeX;
    private double relativeY;
    private double relativeZ;
    private double relativePitch;
    private double relativeYaw;
    private double relativeRoll = 0;

    private PhotonCamera photonCamera;

    public RobotCamera(String cameraName, double x, double y, double z, double pitch, double yaw) {
        name = cameraName;
        relativeX = x;
        relativeY = y;
        relativeZ = z;
        relativePitch = pitch;
        relativeYaw = yaw;

        photonCamera = new PhotonCamera(cameraName);
    }

    public Pose3d getRelativePose() {
        return new Pose3d(relativeX, relativeY, relativeZ, new Rotation3d(relativePitch, relativeYaw, relativeRoll));
    }
    public Transform3d getRelativeTransform() {
        return new Transform3d(relativeX, relativeY, relativeZ, new Rotation3d(relativePitch, relativeYaw, relativeRoll));
    }

    public PhotonPipelineResult getLatestResult() {
        return photonCamera.getLatestResult();
    }

    public double getRelativeX() {
        return relativeX;
    }

}
