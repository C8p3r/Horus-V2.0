package frc.robot.constants;

/**
 * Constants for the CANdle LED subsystem
 * 
 * AUTOMATIC GRB COLOR REMAPPING:
 * ===============================
 * This subsystem automatically handles GRB color order in SOFTWARE.
 * No Phoenix Tuner X configuration needed!
 * 
 * How it works:
 * - You call setRGB(255, 0, 0) for RED in your code
 * - The subsystem automatically swaps channels: (R, G, B) -> (G, R, B)
 * - The strip receives (0, 255, 0) which displays RED on GRB strips
 * 
 * Color Mapping (RGB input -> GRB output):
 * - setRGB(255, 0, 0) -> RED displays correctly
 * - setRGB(0, 255, 0) -> GREEN displays correctly
 * - setRGB(0, 0, 255) -> BLUE displays correctly
 * - setRGB(255, 255, 0) -> YELLOW displays correctly
 * - etc.
 * 
 * This works for WS2812B, NeoPixel, and other GRB LED strips.
 * No hardware configuration required - handled in code!
 */
public final class CANdleConstants {
    
    // CAN Configuration
    public static final String CANBUS_NAME = "Drivetrain";
    public static final int CANDLE_ID = 13;
    
    // LED Configuration
    public static final int LED_START_INDEX = 0; // Start at first onboard LED
    public static final int LED_END_INDEX = 99; // 8 onboard + 60 external = 68 total (0-67)
    
    // LED Counts
    public static final int NUM_ONBOARD_LEDS = 8;
    public static final int NUM_EXTERNAL_LEDS = 60;
    public static final int TOTAL_LED_COUNT = NUM_ONBOARD_LEDS + NUM_EXTERNAL_LEDS;
    
    // Brightness Settings (0.0 to 1.0)
    public static final double DEFAULT_BRIGHTNESS = 0.5;
    public static final double DIM_BRIGHTNESS = 0.2;
    public static final double BRIGHT_BRIGHTNESS = 1.0;
    
    // Animation Settings
    public static final double ANIMATION_SPEED = 0.5; // Speed multiplier
    public static final int ANIMATION_SLOT = 0; // Slot for storing animations
}
