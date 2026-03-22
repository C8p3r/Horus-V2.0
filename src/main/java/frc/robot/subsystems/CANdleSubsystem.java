package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CANdleConfiguration;
import com.ctre.phoenix6.controls.FireAnimation;
import com.ctre.phoenix6.controls.LarsonAnimation;
import com.ctre.phoenix6.controls.RainbowAnimation;
import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.controls.StrobeAnimation;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.AnimationDirectionValue;
import com.ctre.phoenix6.signals.LarsonBounceValue;
import com.ctre.phoenix6.signals.RGBWColor;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.CANdleConstants;

/**
 * Two-Zone LED Control System
 * 
 * ZONE 1 - CANdle (LEDs 0-7): Debugging feedback
 * - Rainbow idle when all systems at rest
 * - Green 1-LED Larson when intake active
 * - Yellow 1-LED Larson when outtaking
 * - Blue strobe when shooting active
 * 
 * ZONE 2 - Lightbar (LEDs 8-35): Visual feedback
 * - RGB cycle when disabled
 * - White Larson when idle (size 15)
 * - Green Larson when intaking (size 15)
 * - Yellow Larson when outtaking (size 15)
 * - Fire animation when shooting (intensity scales with flywheel, blue when at speed)
 */
public class CANdleSubsystem extends SubsystemBase {
    // Hardware
    private final CANBus canBus;
    private final CANdle candle;
    
    // Zone definitions
    private static final int CANDLE_START = 0;
    private static final int CANDLE_END = 7;
    private static final int CANDLE_COUNT = 8; 
    private static final int LIGHTBAR_START = 8;
    private static final int LIGHTBAR_END = 35;
    private static final int LIGHTBAR_COUNT = 35; 
    
    // ZONE 1 - CANdle debugging animations
    private final RainbowAnimation candleRainbow = new RainbowAnimation(CANDLE_START, CANDLE_COUNT)
        .withSlot(0)
        .withBrightness(0.8)
        .withDirection(AnimationDirectionValue.Forward)
        .withFrameRate(Hertz.of(100));
    
    private final LarsonAnimation candleIntake = new LarsonAnimation(CANDLE_START, CANDLE_COUNT)
        .withSlot(0)
        .withColor(new RGBWColor(0, 255, 0, 0)) // Green
        .withSize(1) // Single LED
        .withBounceMode(LarsonBounceValue.Front)
        .withFrameRate(Hertz.of(80));
    
    private final LarsonAnimation candleOuttake = new LarsonAnimation(CANDLE_START, CANDLE_COUNT)
        .withSlot(0)
        .withColor(new RGBWColor(255, 255, 0, 0)) // Yellow
        .withSize(1) // Single LED
        .withBounceMode(LarsonBounceValue.Front)
        .withFrameRate(Hertz.of(80));
    
    private final StrobeAnimation candleShooting = new StrobeAnimation(CANDLE_START, CANDLE_COUNT)
        .withSlot(0)
        .withColor(new RGBWColor(0, 100, 255, 0)) // Blue
        .withFrameRate(Hertz.of(20));
    
    // ZONE 2 - Lightbar visual animations (GRB color order for external strips)
    private final RainbowAnimation lightbarDisabled = new RainbowAnimation(LIGHTBAR_START, LIGHTBAR_COUNT)
        .withSlot(1)
        .withBrightness(0.5)
        .withDirection(AnimationDirectionValue.Forward)
        .withFrameRate(Hertz.of(60));
    
    private final LarsonAnimation lightbarIdle = new LarsonAnimation(LIGHTBAR_START, LIGHTBAR_COUNT)
        .withSlot(1)
        .withColor(new RGBWColor(253, 255, 255, 0)) // White (GRB: 255,253,255)
        .withSize(15)
        .withBounceMode(LarsonBounceValue.Front)
        .withFrameRate(Hertz.of(63));
    
    private final LarsonAnimation lightbarIntake = new LarsonAnimation(LIGHTBAR_START, LIGHTBAR_COUNT)
        .withSlot(1)
        .withColor(new RGBWColor(255, 0, 0, 0)) // Green (GRB: 0,255,0) - swapped R and G
        .withSize(15)
        .withBounceMode(LarsonBounceValue.Front)
        .withFrameRate(Hertz.of(63));
    
    private final LarsonAnimation lightbarOuttake = new LarsonAnimation(LIGHTBAR_START, LIGHTBAR_COUNT)
        .withSlot(1)
        .withColor(new RGBWColor(255, 255, 0, 0)) // Yellow (GRB: 255,255,0) - R and G both on
        .withSize(15)
        .withBounceMode(LarsonBounceValue.Front)
        .withFrameRate(Hertz.of(63));
    
    private final FireAnimation lightbarFire = new FireAnimation(LIGHTBAR_START, LIGHTBAR_COUNT)
        .withSlot(1)
        .withBrightness(1.0)
        .withDirection(AnimationDirectionValue.Forward)
        .withSparking(0.522)
        .withCooling(0.286)
        .withFrameRate(Hertz.of(43));
    
    private final FireAnimation lightbarFireBlue = new FireAnimation(LIGHTBAR_START, LIGHTBAR_COUNT)
        .withSlot(1)
        .withBrightness(1.0)
        .withDirection(AnimationDirectionValue.Forward)
        .withSparking(0.522)
        .withCooling(0.286)
        .withFrameRate(Hertz.of(43));
    
    // State tracking
    private enum RobotState {
        DISABLED,
        IDLE,
        INTAKING,
        OUTTAKING,
        SHOOTING_SPINUP,
        SHOOTING_READY
    }
    
    private RobotState currentState = RobotState.IDLE;
    private RobotState lastAppliedState = null; // Track last applied state to avoid redundant updates
    private double currentFlywheelProgress = 0.0;
    private double lastAppliedProgress = -1.0; // Track last applied progress
    
    public CANdleSubsystem() {
        canBus = new CANBus(CANdleConstants.CANBUS_NAME);
        candle = new CANdle(CANdleConstants.CANDLE_ID, canBus);
        
        // Configure CANdle
        CANdleConfiguration config = new CANdleConfiguration();
        candle.getConfigurator().apply(config);
        
        // Set initial state
        setIdle();
    }
    
    /**
     * Set LED state to idle (all systems at rest)
     * CANdle: Rainbow
     * Lightbar: White Larson
     */
    public void setIdle() {
        currentState = RobotState.IDLE;
    }
    
    /**
     * Set LED state to intaking
     * CANdle: Green 1-LED Larson
     * Lightbar: Green Larson (size 15)
     */
    public void setIntaking() {
        currentState = RobotState.INTAKING;
    }
    
    /**
     * Set LED state to outtaking/ejecting
     * CANdle: Yellow 1-LED Larson
     * Lightbar: Yellow Larson (size 15)
     */
    public void setOuttaking() {
        currentState = RobotState.OUTTAKING;
    }
    
    /**
     * Set LED state to shooting (spinning up)
     * CANdle: Blue strobe
     * Lightbar: Fire animation (normal color)
     * @param flywheelProgress Progress from 0.0 to 1.0 (used to scale intensity)
     */
    public void setShootingSpinup(double flywheelProgress) {
        currentState = RobotState.SHOOTING_SPINUP;
        currentFlywheelProgress = flywheelProgress;
    }
    
    /**
     * Set LED state to shooting (at speed)
     * CANdle: Blue strobe
     * Lightbar: Blue fire animation
     */
    public void setShootingReady() {
        currentState = RobotState.SHOOTING_READY;
    }
    
    /**
     * Set LED state to disabled
     * CANdle: Rainbow (slower)
     * Lightbar: RGB cycle
     */
    public void setDisabled() {
        currentState = RobotState.DISABLED;
    }
    
    @Override
    public void periodic() {
        // Auto-detect disabled state
        if (DriverStation.isDisabled() && currentState != RobotState.DISABLED) {
            setDisabled();
        }
        
        // Only update LEDs when state changes or progress changes significantly
        boolean stateChanged = currentState != lastAppliedState;
        boolean progressChanged = Math.abs(currentFlywheelProgress - lastAppliedProgress) > 0.05; // 5% threshold
        
        if (!stateChanged && !progressChanged) {
            return; // No update needed, prevent flickering
        }
        
        // Apply animations based on current state
        switch (currentState) {
            case DISABLED:
                if (stateChanged) {
                    candle.setControl(candleRainbow);
                    candle.setControl(lightbarDisabled);
                }
                break;
                
            case IDLE:
                if (stateChanged) {
                    candle.setControl(candleRainbow);
                    candle.setControl(lightbarIdle);
                }
                break;
                
            case INTAKING:
                if (stateChanged) {
                    candle.setControl(candleIntake);
                    candle.setControl(lightbarIntake);
                }
                break;
                
            case OUTTAKING:
                if (stateChanged) {
                    candle.setControl(candleOuttake);
                    candle.setControl(lightbarOuttake);
                }
                break;
                
            case SHOOTING_SPINUP:
                if (stateChanged || progressChanged) {
                    candle.setControl(candleShooting);
                    // Create scaled fire animation based on flywheel progress
                    FireAnimation scaledFire = new FireAnimation(LIGHTBAR_START, LIGHTBAR_COUNT)
                        .withSlot(1)
                        .withBrightness(Math.max(0.3, currentFlywheelProgress))
                        .withDirection(AnimationDirectionValue.Forward)
                        .withSparking(0.522)
                        .withCooling(0.286)
                        .withFrameRate(Hertz.of(43));
                    candle.setControl(scaledFire);
                    lastAppliedProgress = currentFlywheelProgress;
                }
                break;
                
            case SHOOTING_READY:
                if (stateChanged) {
                    candle.setControl(candleShooting);
                    candle.setControl(lightbarFireBlue);
                }
                break;
        }
        
        lastAppliedState = currentState;
    }
    
    /**
     * Get current LED state
     */
    public RobotState getState() {
        return currentState;
    }
}
