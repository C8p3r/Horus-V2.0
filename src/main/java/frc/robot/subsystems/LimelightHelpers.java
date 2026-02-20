package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

import java.util.ArrayList;
import java.util.List;

/**
 * Helper class for interfacing with Limelight Network Tables
 * Supports MegaTag2 with full 3D localization
 */
public class LimelightHelpers {
    
    /**
     * MegaTag2 Pose Estimate Result with full metadata
     */
    public static class PoseEstimate {
        public Pose2d pose;
        public Pose3d pose3d;
        public double timestampSeconds;
        public double latency;
        public int tagCount;
        public double tagSpan;
        public double avgTagDist;
        public double avgTagArea;

        public PoseEstimate() {
            this.pose = new Pose2d();
            this.pose3d = new Pose3d();
            this.timestampSeconds = 0;
            this.latency = 0;
            this.tagCount = 0;
            this.tagSpan = 0;
            this.avgTagDist = 0;
            this.avgTagArea = 0;
        }

        public PoseEstimate(Pose2d pose, Pose3d pose3d, double timestampSeconds, double latency,
                          int tagCount, double tagSpan, double avgTagDist, double avgTagArea) {
            this.pose = pose;
            this.pose3d = pose3d;
            this.timestampSeconds = timestampSeconds;
            this.latency = latency;
            this.tagCount = tagCount;
            this.tagSpan = tagSpan;
            this.avgTagDist = avgTagDist;
            this.avgTagArea = avgTagArea;
        }
    }
    
    // Helper methods to convert arrays to Pose objects
    private static Pose3d toPose3D(double[] inData) {
        if (inData.length < 6) {
            return new Pose3d();
        }
        return new Pose3d(
            new Translation3d(inData[0], inData[1], inData[2]),
            new Rotation3d(Math.toRadians(inData[3]), Math.toRadians(inData[4]), Math.toRadians(inData[5]))
        );
    }

    private static Pose2d toPose2D(double[] inData) {
        if (inData.length < 6) {
            return new Pose2d();
        }
        Translation2d translation = new Translation2d(inData[0], inData[1]);
        Rotation2d rotation = Rotation2d.fromDegrees(inData[5]);
        return new Pose2d(translation, rotation);
    }
    
    /**
     * Gets the Limelight NetworkTable
     */
    private static NetworkTable getLimelightNTTable(String tableName) {
        return NetworkTableInstance.getDefault().getTable(tableName);
    }
    
    /**
     * Gets a NetworkTable entry
     */
    private static NetworkTableEntry getLimelightNTTableEntry(String tableName, String entryName) {
        return getLimelightNTTable(tableName).getEntry(entryName);
    }
    
    /**
     * Gets a double value from NetworkTables
     */
    private static double getLimelightNTDouble(String tableName, String entryName) {
        return getLimelightNTTableEntry(tableName, entryName).getDouble(0.0);
    }
    
    /**
     * Gets a double array from NetworkTables
     */
    private static double[] getLimelightNTDoubleArray(String tableName, String entryName) {
        return getLimelightNTTableEntry(tableName, entryName).getDoubleArray(new double[0]);
    }
    
    /**
     * Whether the limelight has a valid target
     */
    public static boolean hasTarget(String limelightName) {
        return getLimelightNTDouble(limelightName, "tv") == 1.0;
    }
    
    /**
     * Gets the latency contribution from the Limelight (in milliseconds)
     */
    public static double getLatency_Pipeline(String limelightName) {
        return getLimelightNTDouble(limelightName, "tl");
    }
    
    /**
     * Gets the latency contribution from image capture (in milliseconds)
     */
    public static double getLatency_Capture(String limelightName) {
        return getLimelightNTDouble(limelightName, "cl");
    }
    
    /**
     * Gets the total latency (pipeline + capture in milliseconds)
     */
    public static double getTotalLatency(String limelightName) {
        return getLatency_Pipeline(limelightName) + getLatency_Capture(limelightName);
    }
    
    /**
     * Gets the MegaTag2 robot pose estimate in field space (WPI Blue Alliance)
     * This is the RECOMMENDED method for getting pose with MegaTag2
     * Returns full PoseEstimate with tag count, tag span, distance, and area metadata
     */
    public static PoseEstimate getBotPoseEstimate_wpiBlue(String limelightName) {
        return getBotPoseEstimate(limelightName, "botpose_wpiblue");
    }
    
    /**
     * Gets the MegaTag2 robot pose estimate in field space (WPI Red Alliance)
     * This is the RECOMMENDED method for getting pose with MegaTag2
     * Returns full PoseEstimate with tag count, tag span, distance, and area metadata
     */
    public static PoseEstimate getBotPoseEstimate_wpiRed(String limelightName) {
        return getBotPoseEstimate(limelightName, "botpose_wpired");
    }
    
    /**
     * Gets the MegaTag2 robot pose estimate with full metadata
     * MegaTag2 provides:
     * - 2D and 3D pose
     * - Tag count (number of AprilTags used in pose)
     * - Tag span (spread of tags, larger = better geometry)
     * - Average distance to tags
     * - Average tag area (larger = closer/more reliable)
     */
    private static PoseEstimate getBotPoseEstimate(String limelightName, String entry) {
        double[] poseArray = getLimelightNTDoubleArray(limelightName, entry);
        
        // MegaTag2 botpose format:
        // [0-5]: X, Y, Z, Roll, Pitch, Yaw (robot pose in field space)
        // [6]: Total latency (ms)
        // [7]: Tag count
        // [8]: Tag span (meters)
        // [9]: Average tag distance (meters)
        // [10]: Average tag area (% of image, 0-100)
        
        if (poseArray.length < 11) {
            return new PoseEstimate();
        }
        
        Pose2d pose2d = toPose2D(poseArray);
        Pose3d pose3d = toPose3D(poseArray);
        double latency = poseArray[6];
        int tagCount = (int)poseArray[7];
        double tagSpan = poseArray[8];
        double avgTagDist = poseArray[9];
        double avgTagArea = poseArray[10];
        
        double timestampSeconds = (edu.wpi.first.wpilibj.Timer.getFPGATimestamp() - (latency / 1000.0));
        
        return new PoseEstimate(pose2d, pose3d, timestampSeconds, latency, tagCount, tagSpan, avgTagDist, avgTagArea);
    }
    
    /**
     * Gets the robot pose in field space using MegaTag2 (blue alliance origin)
     * Returns the pose from botpose_wpiblue
     */
    public static Pose2d getBotPose2d_wpiBlue(String limelightName) {
        double[] poseArray = getLimelightNTDoubleArray(limelightName, "botpose_wpiblue");
        if (poseArray.length < 6) {
            return new Pose2d();
        }
        return new Pose2d(
            new Translation2d(poseArray[0], poseArray[1]),
            Rotation2d.fromDegrees(poseArray[5])
        );
    }
    
    /**
     * Gets the robot pose in field space using MegaTag2 (red alliance origin)
     * Returns the pose from botpose_wpired
     */
    public static Pose2d getBotPose2d_wpiRed(String limelightName) {
        double[] poseArray = getLimelightNTDoubleArray(limelightName, "botpose_wpired");
        if (poseArray.length < 6) {
            return new Pose2d();
        }
        return new Pose2d(
            new Translation2d(poseArray[0], poseArray[1]),
            Rotation2d.fromDegrees(poseArray[5])
        );
    }
    
    /**
     * Gets the 3D robot pose in field space
     */
    public static Pose3d getBotPose3d_wpiBlue(String limelightName) {
        double[] poseArray = getLimelightNTDoubleArray(limelightName, "botpose_wpiblue");
        if (poseArray.length < 6) {
            return new Pose3d();
        }
        return new Pose3d(
            poseArray[0], poseArray[1], poseArray[2],
            new edu.wpi.first.math.geometry.Rotation3d(
                Math.toRadians(poseArray[3]),
                Math.toRadians(poseArray[4]),
                Math.toRadians(poseArray[5])
            )
        );
    }
    
    /**
     * Gets the number of AprilTags currently visible
     */
    public static int getNumberOfAprilTags(String limelightName) {
        double[] botpose = getLimelightNTDoubleArray(limelightName, "botpose_wpiblue");
        if (botpose.length < 8) {
            return 0;
        }
        return (int) botpose[7]; // Index 7 contains the tag count
    }
    
    /**
     * Gets the average distance to visible AprilTags (in meters)
     */
    public static double getAverageTagDistance(String limelightName) {
        double[] botpose = getLimelightNTDoubleArray(limelightName, "botpose_wpiblue");
        if (botpose.length < 9) {
            return 0.0;
        }
        return botpose[8]; // Index 8 contains average tag distance
    }
    
    /**
     * Gets the pose ambiguity (lower is better, 0-1 range typically)
     */
    public static double getPoseAmbiguity(String limelightName) {
        double[] botpose = getLimelightNTDoubleArray(limelightName, "botpose_wpiblue");
        if (botpose.length < 10) {
            return 1.0;
        }
        return botpose[9]; // Index 9 contains pose ambiguity
    }
    
    /**
     * Sets the LED mode of the Limelight
     * 0 = use pipeline mode, 1 = force off, 2 = force blink, 3 = force on
     */
    public static void setLEDMode(String limelightName, int mode) {
        getLimelightNTTable(limelightName).getEntry("ledMode").setNumber(mode);
    }
    
    /**
     * Sets the pipeline index
     */
    public static void setPipeline(String limelightName, int pipeline) {
        getLimelightNTTable(limelightName).getEntry("pipeline").setNumber(pipeline);
    }
    
    /**
     * Gets the camera pose in 3D space (field coordinates)
     * This is calculated as: robot pose + camera offset
     */
    public static Pose3d getCameraPose3d(String limelightName, Pose3d robotPose, Transform3d cameraToRobot) {
        return robotPose.transformBy(cameraToRobot.inverse());
    }
    
    /**
     * Gets the 3D poses of all visible AprilTags in field space
     * Uses the targetpose_robotspace data from NetworkTables
     */
    public static Pose3d[] getVisibleTagPoses3d(String limelightName, Pose3d robotPose) {
        // Get JSON results which contain individual tag data
        // For now, we'll use a simpler approach with the tag IDs and calculate positions
        // The Limelight publishes tid (tag ID) for detected tags
        
        double[] tidArray = getLimelightNTDoubleArray(limelightName, "tid");
        double[] targetPoseRobotSpace = getLimelightNTDoubleArray(limelightName, "targetpose_robotspace");
        
        List<Pose3d> tagPoses = new ArrayList<>();
        
        // If we have tag ID data and target pose data
        if (tidArray.length > 0 && targetPoseRobotSpace.length >= 6) {
            // targetpose_robotspace gives us the tag position relative to robot
            // We need to transform it to field space
            Pose3d tagInRobotSpace = toPose3D(targetPoseRobotSpace);
            Pose3d tagInFieldSpace = robotPose.transformBy(new Transform3d(
                tagInRobotSpace.getTranslation(),
                tagInRobotSpace.getRotation()
            ));
            tagPoses.add(tagInFieldSpace);
        }
        
        // Note: For multiple tags, Limelight uses JSON data
        // This is a simplified version that works for single tag or averaged position
        // For full multi-tag support, you'd need to parse the JSON results
        
        return tagPoses.toArray(new Pose3d[0]);
    }
    
    /**
     * Gets tag IDs of all currently visible tags
     */
    public static int[] getVisibleTagIDs(String limelightName) {
        double[] tidArray = getLimelightNTDoubleArray(limelightName, "tid");
        int[] tagIDs = new int[tidArray.length];
        for (int i = 0; i < tidArray.length; i++) {
            tagIDs[i] = (int) tidArray[i];
        }
        return tagIDs;
    }
}
