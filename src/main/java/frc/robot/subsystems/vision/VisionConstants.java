package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;

public class VisionConstants {
    // AprilTag layout
    public static AprilTagFieldLayout aprilTagLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);

    // Camera names, must match names configured on coprocessor
    public static String cam0 = "limelight-l";
    public static String cam1 = "limelight-r";

    // Robot to camera transforms
    // (Not used by Limelight, configure in web UI instead)
    private static double inchesToMeters = 39.37;
    public static Transform3d robotToCamera0 = new Transform3d(-9.7366 / inchesToMeters, -13.0 / inchesToMeters, 8.25 / inchesToMeters, new Rotation3d(0.0, Math.toRadians(-20.0), Math.toRadians(-90.0)));
    public static Transform3d robotToCamera1 = new Transform3d(-12.5 / inchesToMeters, -11.1361 / inchesToMeters, 8.25 / inchesToMeters, new Rotation3d(0.0, Math.toRadians(-20.0), Math.toRadians(-158.0)));
    public static Transform3d robotToCamera2 = new Transform3d(-12.5 / inchesToMeters, 11.1361 / inchesToMeters, 8.25 / inchesToMeters, new Rotation3d(0.0, Math.toRadians(-20.0), Math.toRadians(158.0)));
    public static Transform3d robotToCamera3 = new Transform3d(-9.7366 / inchesToMeters, 13.0 / inchesToMeters, 8.25 / inchesToMeters, new Rotation3d(0.0, Math.toRadians(-20.0), Math.toRadians(90.0)));

    // Basic filtering thresholds
    public static double maxAmbiguity = 0.3;
    public static double maxZError = 0.75;

    // Standard deviation baselines, for 1 meter distance and 1 tag
    // (Adjusted automatically based on distance and # of tags)
    public static double linearStdDevBaseline = 0.16; // Meters (was 0.02)
    public static double angularStdDevBaseline = 0.26; // Radians (was 0.06)

    // Standard deviation multipliers for each camera
    // (Adjust to trust some cameras more than others)
    public static double[] cameraStdDevFactors = new double[] {};

    // Multipliers to apply for MegaTag 2 observations
    public static double linearStdDevMegatag2Factor = 0.5; // More stable than full 3D solve
    public static double angularStdDevMegatag2Factor = Double.POSITIVE_INFINITY; // No rotation data available
}
