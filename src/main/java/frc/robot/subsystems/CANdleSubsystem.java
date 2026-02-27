package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CANdleConfiguration;
import com.ctre.phoenix6.controls.SingleFadeAnimation;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.RGBWColor;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.CANdleConstants;
import frc.robot.util.TelemetryThrottle;

/**
 * CANdle LED subsystem with breathing animation
 * 
 * AUTOMATIC GRB REMAPPING FOR WS2812B/NEOPIXEL STRIPS:
 * This subsystem automatically converts RGB input to GRB output in software.
 * You can call setRGB(255, 0, 0) for red, and it will correctly display red
 * on GRB LED strips by internally swapping the channels.
 * 
 * Color Channel Mapping:
 * - Your RED input (R) -> Sent as GREEN channel to strip
 * - Your GREEN input (G) -> Sent as RED channel to strip  
 * - Your BLUE input (B) -> Sent as BLUE channel to strip
 * 
 * This means:
 * - setRGB(255, 0, 0) displays RED (sent as 0, 255, 0 to GRB strip)
 * - setRGB(0, 255, 0) displays GREEN (sent as 255, 0, 0 to GRB strip)
 * - setRGB(0, 0, 255) displays BLUE (sent as 0, 0, 255 to GRB strip)
 * 
 * NO PHOENIX TUNER CONFIGURATION NEEDED - handled in software!
 * Just make sure your CANdle firmware is up to date.
 */
public class CANdleSubsystem extends SubsystemBase {
    private final CANBus canBus;
    private final CANdle candle;
    private final TelemetryThrottle telemetryThrottle;
    private SingleFadeAnimation currentAnimation;
    
    // Track current LED state for dashboard visualization
    private int currentRed = 255;
    private int currentGreen = 255;
    private int currentBlue = 255;
    private String currentMode = "Idle";
    
    public CANdleSubsystem() {
        canBus = new CANBus(CANdleConstants.CANBUS_NAME);
        candle = new CANdle(CANdleConstants.CANDLE_ID, canBus);
        
        // Throttle telemetry to 5 Hz (200ms) to prevent network I/O blocking
        telemetryThrottle = new TelemetryThrottle(0.2);
        
        // Configure CANdle with basic settings
        // Note: Phoenix 6 CANdle API uses RGB by default
        // LED strip type is auto-detected by the CANdle hardware
        CANdleConfiguration config = new CANdleConfiguration();
        candle.getConfigurator().apply(config);
    }
    
    /**
     * Sets solid color on all LEDs
     * Automatically remaps RGB input to GRB output for WS2812B/NeoPixel strips
     * @param r Red value (0-255) - sent as Green to strip
     * @param g Green value (0-255) - sent as Red to strip
     * @param b Blue value (0-255) - sent as Blue to strip
     */
    public void setRGB(int r, int g, int b) {
        currentRed = r;
        currentGreen = g;
        currentBlue = b;
        currentMode = "Solid";
        
        // Remap RGB to GRB: R->G, G->R, B->B
        candle.setControl(
            new com.ctre.phoenix6.controls.SolidColor(0, CANdleConstants.TOTAL_LED_COUNT)
                .withColor(new RGBWColor(g, r, b)) // Swapped: send (G, R, B)
        );
    }
    
    /**
     * Sets breathing/pulsing color with hardware-accelerated animation
     * Automatically remaps RGB input to GRB output for WS2812B/NeoPixel strips
     * @param r Red value (0-255) - sent as Green to strip
     * @param g Green value (0-255) - sent as Red to strip
     * @param b Blue value (0-255) - sent as Blue to strip
     * @param speed Breathing speed (Hz) - higher = faster breathing
     * @param minBright Minimum brightness (0-255)
     * @param maxBright Maximum brightness (0-255)
     */
    public void setRGBWithModulation(int r, int g, int b, double speed, double minBright, double maxBright) {
        currentRed = r;
        currentGreen = g;
        currentBlue = b;
        currentMode = "Breathing";
        
        // Remap RGB to GRB: R->G, G->R, B->B
        // Create breathing animation
        currentAnimation = new SingleFadeAnimation(0, CANdleConstants.TOTAL_LED_COUNT)
            .withSlot(0)
            .withColor(new RGBWColor(g, r, b)) // Swapped: send (G, R, B)
            .withFrameRate(Hertz.of(speed * 100)); // 10x faster: speed * 100
        
        candle.setControl(currentAnimation);
    }
    
    /**
     * Convenience method with default modulation (20Hz breathing - 10x faster)
     */
    public void setRGBWithModulation(int r, int g, int b) {
        setRGBWithModulation(r, g, b, 2.0, 50, 255);
    }
    
    /**
     * Sets rapid red blinking for emergency/overheat warning
     * Automatically remaps to GRB output
     * Blinks at 10Hz (10 times per second)
     */
    public void setRapidRedBlink() {
        currentRed = 255;
        currentGreen = 0;
        currentBlue = 0;
        currentMode = "ALERT";
        
        // Remap RGB to GRB: Red(255,0,0) -> send as (0,255,0) for GRB strips
        // Use SingleFadeAnimation for rapid blinking effect
        currentAnimation = new SingleFadeAnimation(0, CANdleConstants.TOTAL_LED_COUNT)
            .withSlot(0)
            .withColor(new RGBWColor(0, 255, 0)) // Swapped: send (G=0, R=255, B=0) for red
            .withFrameRate(Hertz.of(1000)); // Very fast for rapid blink effect
        
        candle.setControl(currentAnimation);
    }
    
    /**
     * Sets slow rainbow pattern (climber deployed - waiting)
     */
    public void setSlowRainbow() {
        currentMode = "Slow Rainbow";
        
        // Use RainbowAnimation for smooth rainbow effect
        candle.setControl(
            new com.ctre.phoenix6.controls.RainbowAnimation(0, CANdleConstants.TOTAL_LED_COUNT)
                .withSlot(0)
                .withBrightness(1.0)
        );
    }
    
    /**
     * Sets rapid rainbow pattern (actively climbing)
     * Uses the same rainbow but will be perceived as "active" during climbing
     */
    public void setRapidRainbow() {
        currentMode = "Rapid Rainbow";
        
        // Use RainbowAnimation - same effect, different context (actively climbing)
        candle.setControl(
            new com.ctre.phoenix6.controls.RainbowAnimation(0, CANdleConstants.TOTAL_LED_COUNT)
                .withSlot(0)
                .withBrightness(1.0)
        );
    }
    
    @Override
    public void periodic() {
        // Apply animation each cycle if it exists
        if (currentAnimation != null) {
            candle.setControl(currentAnimation);
        }
        
        // Throttle telemetry updates to prevent Status loop blocking
        if (!telemetryThrottle.shouldUpdate()) {
            return; // Skip this update cycle
        }
        
        // Publish CANdle status to SmartDashboard for visualization (throttled to 5 Hz)
        SmartDashboard.putString("CANdle/Mode", currentMode);
        SmartDashboard.putNumber("CANdle/Red", currentRed);
        SmartDashboard.putNumber("CANdle/Green", currentGreen);
        SmartDashboard.putNumber("CANdle/Blue", currentBlue);
        
        // Create a color string for easier visualization
        String colorHex = String.format("#%02X%02X%02X", currentRed, currentGreen, currentBlue);
        SmartDashboard.putString("CANdle/Color (Hex)", colorHex);
        
        // Publish a visual indicator (colored text representation)
        String colorBar = "█████████████████████";
        SmartDashboard.putString("CANdle/Visual", colorBar + " " + getColorName());
    }
    
    /**
     * Get a human-readable color name based on current RGB values
     */
    private String getColorName() {
        if (currentRed > 200 && currentGreen < 100 && currentBlue < 100) {
            return "RED";
        } else if (currentRed < 100 && currentGreen > 200 && currentBlue < 100) {
            return "GREEN";
        } else if (currentRed < 100 && currentGreen < 100 && currentBlue > 200) {
            return "BLUE";
        } else if (currentRed > 200 && currentGreen > 200 && currentBlue < 100) {
            return "YELLOW";
        } else if (currentRed > 200 && currentGreen < 100 && currentBlue > 200) {
            return "PURPLE";
        } else if (currentRed > 150 && currentGreen > 100 && currentBlue < 100) {
            return "ORANGE";
        } else if (currentRed > 200 && currentGreen > 200 && currentBlue > 200) {
            return "WHITE";
        } else {
            return "Custom";
        }
    }
}
