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

    private NetworkTable table;

    private PhotonCamera camera = new PhotonCamera("Camera");

    public void periodic() {
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
        if(photonVisionLatestResult.hasTargets()){
            SmartDashboard.putNumber("Photon Vision", photonVisionLatestResult.getBestTarget().getYaw());
        }
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

}
