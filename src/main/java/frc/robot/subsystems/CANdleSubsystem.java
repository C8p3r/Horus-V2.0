package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CANdleConfiguration;
import com.ctre.phoenix6.controls.SingleFadeAnimation;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.RGBWColor;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.CANdleConstants;

/**
 * Simplified CANdle LED subsystem with breathing animation
 */
public class CANdleSubsystem extends SubsystemBase {
    private final CANBus canBus;
    private final CANdle candle;
    private SingleFadeAnimation currentAnimation;
    
    public CANdleSubsystem() {
        canBus = new CANBus(CANdleConstants.CANBUS_NAME);
        candle = new CANdle(CANdleConstants.CANDLE_ID, canBus);
        
        // Apply simple config
        CANdleConfiguration config = new CANdleConfiguration();
        candle.getConfigurator().apply(config);
    }
    
    /**
     * Sets solid color on all LEDs
     */
    public void setRGB(int r, int g, int b) {
        candle.setControl(
            new com.ctre.phoenix6.controls.SolidColor(0, CANdleConstants.TOTAL_LED_COUNT)
                .withColor(new RGBWColor(r, g, b))
        );
    }
    
    /**
     * Sets breathing/pulsing color with hardware-accelerated animation
     * @param speed Breathing speed (Hz) - higher = faster breathing
     * @param minBright Minimum brightness (0-255)
     * @param maxBright Maximum brightness (0-255)
     */
    public void setRGBWithModulation(int r, int g, int b, double speed, double minBright, double maxBright) {
        // Create breathing animation
        currentAnimation = new SingleFadeAnimation(0, CANdleConstants.TOTAL_LED_COUNT)
            .withSlot(0)
            .withColor(new RGBWColor(r, g, b))
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
     * Blinks at 10Hz (10 times per second)
     */
    public void setRapidRedBlink() {
        // Use SingleFadeAnimation for rapid blinking effect
        currentAnimation = new SingleFadeAnimation(0, CANdleConstants.TOTAL_LED_COUNT)
            .withSlot(0)
            .withColor(new RGBWColor(255, 0, 0)) // Bright red
            .withFrameRate(Hertz.of(1000)); // Very fast for rapid blink effect
        
        candle.setControl(currentAnimation);
    }
    
    @Override
    public void periodic() {
        // Apply animation each cycle if it exists
        if (currentAnimation != null) {
            candle.setControl(currentAnimation);
        }
    }
}
