package frc.robot.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;

public class AprilTagFieldLayout {
    public static final double TAG_SIZE = 0.1524; // 6 inches

    // Copied from https://firstfrc.blob.core.windows.net/frc2024/FieldAssets/2024LayoutMarkingDiagram.pdf
    // (page 4)

    /*
        id, x, y, z, angle
        1 593.68 9.68 53.38 120°
        2 637.21 34.79 53.38 120°
        3 652.73 196.17 57.13 180°
        4 652.73 218.42 57.13 180°
        5 578.77 323.00 53.38 270°
        6 72.5 323.00 53.38 270°
        7 -1.50 218.42 57.13 0°
        8 -1.50 196.17 57.13 0°
        9 14.02 34.79 53.38 60°
        10 57.54 9.68 53.38 60°
        11 468.69 146.19 52.00 300°
        12 468.69 177.10 52.00 60°
        13 441.74 161.62 52.00 180°
        14 209.48 161.62 52.00 0°
        15 182.73 177.10 52.00 120°
        16 182.73 146.19 52.00 240°
     */

    public static final Map<Integer, Pose3d> fieldPositions = Map.ofEntries(
        Map.entry(1, new Pose3d(Units.inchesToMeters(593.68), Units.inchesToMeters(9.68), Units.inchesToMeters(53.38), new Rotation3d(Units.degreesToRadians(120), 0, 0))),
        Map.entry(2, new Pose3d(Units.inchesToMeters(637.21), Units.inchesToMeters(34.79), Units.inchesToMeters(53.38), new Rotation3d(Units.degreesToRadians(120), 0, 0))),
        Map.entry(3, new Pose3d(Units.inchesToMeters(652.73), Units.inchesToMeters(196.17), Units.inchesToMeters(57.13), new Rotation3d(Units.degreesToRadians(180), 0, 0))),
        Map.entry(4, new Pose3d(Units.inchesToMeters(652.73), Units.inchesToMeters(218.42), Units.inchesToMeters(57.13), new Rotation3d(Units.degreesToRadians(180), 0, 0))),
        Map.entry(5, new Pose3d(Units.inchesToMeters(578.77), Units.inchesToMeters(323.00), Units.inchesToMeters(53.38), new Rotation3d(Units.degreesToRadians(270), 0, 0))),
        Map.entry(6, new Pose3d(Units.inchesToMeters(72.5), Units.inchesToMeters(323.00), Units.inchesToMeters(53.38), new Rotation3d(Units.degreesToRadians(270), 0, 0))),
        Map.entry(7, new Pose3d(Units.inchesToMeters(-1.50), Units.inchesToMeters(218.42), Units.inchesToMeters(57.13), new Rotation3d(Units.degreesToRadians(0), 0, 0))),
        Map.entry(8, new Pose3d(Units.inchesToMeters(-1.50), Units.inchesToMeters(196.17), Units.inchesToMeters(57.13), new Rotation3d(Units.degreesToRadians(0), 0, 0))),
        Map.entry(9, new Pose3d(Units.inchesToMeters(14.02), Units.inchesToMeters(34.79), Units.inchesToMeters(53.38), new Rotation3d(Units.degreesToRadians(60), 0, 0))),
        Map.entry(10, new Pose3d(Units.inchesToMeters(57.54), Units.inchesToMeters(9.68), Units.inchesToMeters(53.38), new Rotation3d(Units.degreesToRadians(60), 0, 0))),
        Map.entry(11, new Pose3d(Units.inchesToMeters(468.69), Units.inchesToMeters(146.19), Units.inchesToMeters(52.00), new Rotation3d(Units.degreesToRadians(300), 0, 0))),
        Map.entry(12, new Pose3d(Units.inchesToMeters(468.69), Units.inchesToMeters(177.10), Units.inchesToMeters(52.00), new Rotation3d(Units.degreesToRadians(60), 0, 0))),
        Map.entry(13, new Pose3d(Units.inchesToMeters(441.74), Units.inchesToMeters(161.62), Units.inchesToMeters(52.00), new Rotation3d(Units.degreesToRadians(180), 0, 0))),
        Map.entry(14, new Pose3d(Units.inchesToMeters(209.48), Units.inchesToMeters(161.62), Units.inchesToMeters(52.00), new Rotation3d(Units.degreesToRadians(0), 0, 0))),
        Map.entry(15, new Pose3d(Units.inchesToMeters(182.73), Units.inchesToMeters(177.10), Units.inchesToMeters(52.00), new Rotation3d(Units.degreesToRadians(120), 0, 0))),
        Map.entry(16, new Pose3d(Units.inchesToMeters(182.73), Units.inchesToMeters(146.19), Units.inchesToMeters(52.00), new Rotation3d(Units.degreesToRadians(240), 0, 0)))
    );

    public static final List<Pose2d> goalPositions = new ArrayList<>(List.of(
        fieldPositions.get(4).toPose2d(),
        fieldPositions.get(7).toPose2d()
    ));


    public static Pose3d getTagPose(int fiducialId) {
        return fieldPositions.get(fiducialId);
    }
}
