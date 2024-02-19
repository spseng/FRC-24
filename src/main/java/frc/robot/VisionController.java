package frc.robot;

import org.photonvision.PhotonCamera;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;


public class VisionController {
    private double x;
    private double y;
    private double area;
    private double yaw;

    private NetworkTable table;
    private PhotonCamera camera = new PhotonCamera("photonvision");

    public void periodic() {
        updateYaw()
        // This method will be called once per scheduler run

        // NetworkTableEntry tx = table.getEntry("tx");
        // NetworkTableEntry ty = table.getEntry("ty");
        // NetworkTableEntry ta = table.getEntry("ta");

        // //read values periodically
        // double x = tx.getDouble(0.0);
        // double y = ty.getDouble(0.0);
        // double area = ta.getDouble(0.0);

        // post to smart dashboard periodically
        // SmartDashboard.putNumber("LimelightX", x);
        // SmartDashboard.putNumber("LimelightY", y);
        // SmartDashboard.putNumber("LimelightArea", area);


        var photonVisionLatestResult = camera.getLatestResult();
        SmartDashboard.putBoolean("Camera has Target", photonVisionLatestResult.hasTargets());
    
    
        if (photonVisionLatestResult.hasTargets()){
            updateYaw(photonVisionLatestResult);
        }

    
    }

    // PhotonVisionResult may need to be changed
    public updateYaw(PhotonVisionResult photonVisionLatestResult) {
        yaw = photonVisionLatestResult.getBestTarget().getYaw();
        SmartDashboard.putNumber("Photon Vision Yaw", newYaw);
    }

    public VisionController() {

        table = NetworkTableInstance.getDefault().getTable("limelight");
    }


    // Getters and Setters
    public double getX() {
        return x;
    }

    public double getY() {
        return y;
    }

    public double getArea() {
        return area;
    }

    public double getYaw() {
        return yaw;
    }

}
