package frc.robot.util;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Match Timer Utility - Tracks current match period and provides timing information
 * Useful for match-specific behaviors and driver station display
 * 
 * Match Structure (2026 Crescendo):
 * - AUTO:              0:20 - 0:00 (20 seconds)
 * - TRANSITION SHIFT:  2:20 - 2:10 (10 seconds)
 * - SHIFT 1:           2:10 - 1:45 (25 seconds)
 * - SHIFT 2:           1:45 - 1:20 (25 seconds)
 * - SHIFT 3:           1:20 - 0:55 (25 seconds)
 * - SHIFT 4:           0:55 - 0:30 (25 seconds)
 * - END GAME:          0:30 - 0:00 (30 seconds)
 */
public class MatchTimer {
    
    // Match timing constants
    private static final double MATCH_LENGTH = 135.0; // Total match is 2:15 (135 seconds)
    private static final double AUTO_END = 120.0;     // AUTO ends at 2:00
    private static final double SHIFT_1_START = 130.0; // SHIFT 1 starts at 2:10
    private static final double SHIFT_1_END = 105.0;   // SHIFT 1 ends at 1:45
    private static final double SHIFT_2_START = 105.0; // SHIFT 2 starts at 1:45
    private static final double SHIFT_2_END = 80.0;    // SHIFT 2 ends at 1:20
    private static final double SHIFT_3_START = 80.0;  // SHIFT 3 starts at 1:20
    private static final double SHIFT_3_END = 55.0;    // SHIFT 3 ends at 0:55
    private static final double SHIFT_4_START = 55.0;  // SHIFT 4 starts at 0:55
    private static final double SHIFT_4_END = 30.0;    // SHIFT 4 ends at 0:30
    private static final double ENDGAME_START = 30.0;  // ENDGAME starts at 0:30
    
    // Shift change detection
    private static String lastPeriod = "NOT_ACTIVE";
    private static boolean shiftChanged = false;
    
    /**
     * Get the time remaining in the match (in seconds)
     * @return Time remaining or -1 if not in a match
     */
    public static double getTimeRemaining() {
        if (!DriverStation.isEnabled()) {
            return -1.0;
        }
        return DriverStation.getMatchTime();
    }
    
    /**
     * Get the current match period as a string
     * @return Current period name (AUTO, SHIFT_1, SHIFT_2, SHIFT_3, SHIFT_4, ENDGAME, or UNKNOWN)
     */
    public static String getCurrentPeriod() {
        double timeRemaining = getTimeRemaining();
        
        if (timeRemaining < 0) return "NOT_ACTIVE";
        if (timeRemaining > AUTO_END) return "AUTO";
        if (timeRemaining > SHIFT_1_END) return "SHIFT_1";
        if (timeRemaining > SHIFT_2_END) return "SHIFT_2";
        if (timeRemaining > SHIFT_3_END) return "SHIFT_3";
        if (timeRemaining > SHIFT_4_END) return "SHIFT_4";
        if (timeRemaining > 0) return "ENDGAME";
        
        return "UNKNOWN";
    }
    
    /**
     * Check if we're in Autonomous period
     * @return true if in AUTO
     */
    public static boolean isAuto() {
        double timeRemaining = getTimeRemaining();
        return timeRemaining >= 0 && timeRemaining > AUTO_END;
    }
    
    /**
     * Check if we're in Teleop (any shift or endgame)
     * @return true if in TELEOP
     */
    public static boolean isTeleop() {
        return !isAuto();
    }
    
    /**
     * Check if we're in Shift 1
     * @return true if in SHIFT_1
     */
    public static boolean isShift1() {
        double timeRemaining = getTimeRemaining();
        return timeRemaining > SHIFT_1_END && timeRemaining <= SHIFT_1_START;
    }
    
    /**
     * Check if we're in Shift 2
     * @return true if in SHIFT_2
     */
    public static boolean isShift2() {
        double timeRemaining = getTimeRemaining();
        return timeRemaining > SHIFT_2_END && timeRemaining <= SHIFT_2_START;
    }
    
    /**
     * Check if we're in Shift 3
     * @return true if in SHIFT_3
     */
    public static boolean isShift3() {
        double timeRemaining = getTimeRemaining();
        return timeRemaining > SHIFT_3_END && timeRemaining <= SHIFT_3_START;
    }
    
    /**
     * Check if we're in Shift 4
     * @return true if in SHIFT_4
     */
    public static boolean isShift4() {
        double timeRemaining = getTimeRemaining();
        return timeRemaining > SHIFT_4_END && timeRemaining <= SHIFT_4_START;
    }
    
    /**
     * Check if we're in Endgame (last 30 seconds)
     * @return true if in ENDGAME
     */
    public static boolean isEndgame() {
        double timeRemaining = getTimeRemaining();
        return timeRemaining >= 0 && timeRemaining <= ENDGAME_START;
    }
    
    /**
     * Check if we're in the last N seconds of a shift
     * @param secondsLeft Number of seconds left in period
     * @return true if time remaining <= secondsLeft
     */
    public static boolean isLastSeconds(double secondsLeft) {
        double timeRemaining = getTimeRemaining();
        return timeRemaining >= 0 && timeRemaining <= secondsLeft;
    }
    
    /**
     * Format time remaining as MM:SS string for driver station display
     * @return Formatted time string (e.g., "2:45")
     */
    public static String getFormattedTime() {
        double timeRemaining = getTimeRemaining();
        
        if (timeRemaining < 0) {
            return "NOT ACTIVE";
        }
        
        int minutes = (int) timeRemaining / 60;
        int seconds = (int) timeRemaining % 60;
        
        return String.format("%d:%02d", minutes, seconds);
    }
    
    /**
     * Get a detailed status string with period and time
     * @return String like "SHIFT_2 | 1:23"
     */
    public static String getDetailedStatus() {
        return getCurrentPeriod() + " | " + getFormattedTime();
    }
    
    /**
     * Check if a shift change just occurred
     * The shiftChanged boolean toggles between true and false each time we transition to a new shift
     * This allows you to detect shift changes and trigger actions
     * @return true if a shift change was detected (toggles each period)
     */
    public static boolean hasShiftChanged() {
        String currentPeriod = getCurrentPeriod();
        
        // If the period changed, toggle the shift change flag
        if (!currentPeriod.equals(lastPeriod)) {
            lastPeriod = currentPeriod;
            shiftChanged = !shiftChanged;
            return true;
        }
        
        return false;
    }
    
    /**
     * Get the current shift change boolean state
     * Toggles between true and false each time we move to a new shift
     * @return Current state of shift change boolean
     */
    public static boolean getShiftChangedState() {
        return shiftChanged;
    }
    
    /**
     * Update SmartDashboard with match timer information
     * Call this in your main robot periodic or command periodic
     */
    public static void updateDashboard() {
        SmartDashboard.putString("Match/Period", getCurrentPeriod());
        SmartDashboard.putString("Match/Time", getFormattedTime());
        SmartDashboard.putNumber("Match/TimeRemaining", getTimeRemaining());
        SmartDashboard.putBoolean("Match/IsAuto", isAuto());
        SmartDashboard.putBoolean("Match/IsEndgame", isEndgame());
        
        // Detect and publish shift changes
        hasShiftChanged();
        SmartDashboard.putBoolean("Match/ShiftChanged", shiftChanged);
    }
}
