package frc.robot.constants;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;

/**
 * Field target positions for shooting subsystems
 */
public final class FieldConstants {
    
    // Field Dimensions
    private static final double FIELD_LENGTH = 16.54; // meters (blue wall to red wall)
    
    // ==================== HUB TARGETS ====================
    
    /** Blue Alliance Hub - Main scoring target */
    public static final Translation3d BLUE_HUB = new Translation3d(4.7, 4.115, 2.0);
    
    /** Red Alliance Hub - Main scoring target */
    public static final Translation3d RED_HUB = new Translation3d(FIELD_LENGTH - 4.7, 4.115, 2.0);
    
    // ==================== HUMAN PLAYER STATION (HPS) PASS TARGETS ====================
    
    /** Blue HPS Close Pass */
    public static final Translation3d BLUE_HPS_CLOSE_PASS = new Translation3d(3, 1, 0.75);
    
    /** Blue HPS Far Pass */
    public static final Translation3d BLUE_HPS_FAR_PASS = new Translation3d(3, 7, 0.75);
    
    /** Red HPS Close Pass */
    public static final Translation3d RED_HPS_CLOSE_PASS = new Translation3d(FIELD_LENGTH - 3, 1, 0.75);
    
    /** Red HPS Far Pass */
    public static final Translation3d RED_HPS_FAR_PASS = new Translation3d(FIELD_LENGTH - 3, 7, 0.75);
    
    // ==================== ENTRY ANGLE CONSTRAINTS ====================
    
    /** Hub Entry Angle Range */
    public static final double HUB_MIN_ENTRY_ANGLE_DEG = -80.0;  // 80° down
    public static final double HUB_MAX_ENTRY_ANGLE_DEG = 45.0;   // 45° up
    
    /** HPS Pass Entry Angle Range */
    public static final double HPS_MIN_ENTRY_ANGLE_DEG = -90.0;  // Any angle
    public static final double HPS_MAX_ENTRY_ANGLE_DEG = 90.0;   // Any angle
    
    // ==================== MAX SHOOTING DISTANCE CONSTRAINTS ====================
    
    /** Hub Maximum Shooting Distance (meters) */
    public static final double HUB_MAX_SHOOTING_DISTANCE = 4.2;  // 4.2 meters
    
    /** HPS Pass Maximum Distance (meters) */
    public static final double HPS_MAX_SHOOTING_DISTANCE = 15.0;  // 15 meters
    
    // ==================== FIELD POSITION CONSTRAINTS ====================
    
    /** Smart Shoot Maximum X Position */
    public static final double SMART_SHOOT_MAX_X = 4.3;  // Auton-Line
    
    // ==================== DEFAULT TARGET ====================
    
    /** Default shooting target */
    public static final Translation3d DEFAULT_TARGET = BLUE_HUB;
    
    // ==================== HELPER METHODS ====================
    
    /**
     * Determines if a target is a hub
     */
    public static boolean isHubTarget(Translation3d target) {
        return target.equals(RED_HUB) || target.equals(BLUE_HUB);
    }
    
    /**
     * Gets the minimum entry angle for a target
     */
    public static double getMinEntryAngle(Translation3d target) {
        return isHubTarget(target) ? HUB_MIN_ENTRY_ANGLE_DEG : HPS_MIN_ENTRY_ANGLE_DEG;
    }
    
    /**
     * Gets the maximum entry angle for a target
     */
    public static double getMaxEntryAngle(Translation3d target) {
        return isHubTarget(target) ? HUB_MAX_ENTRY_ANGLE_DEG : HPS_MAX_ENTRY_ANGLE_DEG;
    }
    
    /**
     * Gets the maximum shooting distance for a target
     */
    public static double getMaxShootingDistance(Translation3d target) {
        return isHubTarget(target) ? HUB_MAX_SHOOTING_DISTANCE : HPS_MAX_SHOOTING_DISTANCE;
    }
    
    /**
     * Publishes all target positions to NetworkTables
     */
    public static void publishToNetworkTables() {
        Logger.recordOutput("Targets/RedHub", new Pose3d(RED_HUB, new Rotation3d()));
        Logger.recordOutput("Targets/BlueHub", new Pose3d(BLUE_HUB, new Rotation3d()));
        Logger.recordOutput("Targets/RedHPS_ClosePass", new Pose3d(RED_HPS_CLOSE_PASS, new Rotation3d()));
        Logger.recordOutput("Targets/RedHPS_FarPass", new Pose3d(RED_HPS_FAR_PASS, new Rotation3d()));
        Logger.recordOutput("Targets/BlueHPS_ClosePass", new Pose3d(BLUE_HPS_CLOSE_PASS, new Rotation3d()));
        Logger.recordOutput("Targets/BlueHPS_FarPass", new Pose3d(BLUE_HPS_FAR_PASS, new Rotation3d()));
        Logger.recordOutput("Targets/DefaultTarget", new Pose3d(DEFAULT_TARGET, new Rotation3d()));
        Logger.recordOutput("Targets/HubMinEntryAngle", HUB_MIN_ENTRY_ANGLE_DEG);
        Logger.recordOutput("Targets/HubMaxEntryAngle", HUB_MAX_ENTRY_ANGLE_DEG);
        Logger.recordOutput("Targets/HPSMinEntryAngle", HPS_MIN_ENTRY_ANGLE_DEG);
        Logger.recordOutput("Targets/HPSMaxEntryAngle", HPS_MAX_ENTRY_ANGLE_DEG);
    }
}
