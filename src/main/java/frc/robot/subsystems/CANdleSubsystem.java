package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANdleConfiguration;
import com.ctre.phoenix6.configs.LEDConfigs;
import com.ctre.phoenix6.configs.CANdleFeaturesConfigs;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.*;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.CANdleConstants;
import java.util.Optional;

public class CANdleSubsystem extends SubsystemBase {
    private final CANdle candle;
    private final boolean isSimulation;
    
    // Current animation mode tracking
    private AnimationMode currentAnimation = AnimationMode.NONE;
    
    // Simulation state
    private RGBWColor simColor = new RGBWColor(0, 0, 0);
    private double simBrightness = CANdleConstants.DEFAULT_BRIGHTNESS;
    
    /**
     * Animation modes supported by the CANdle
     */
    public enum AnimationMode {
        NONE,
        RAINBOW,
        FIRE,
        LARSON,
        STROBE,
        TWINKLE,
        COLOR_FLOW
    }
    
    /**
     * Creates a new CANdleSubsystem
     */
    public CANdleSubsystem() {
        candle = new CANdle(CANdleConstants.CANDLE_ID, CANdleConstants.CANBUS_NAME);
        isSimulation = RobotBase.isSimulation();
        
        configureCANdle();
        
        // Set default color to off
        setColor(new RGBWColor(0, 0, 0));
    }
    
    /**
     * Configures the CANdle hardware
     */
    private void configureCANdle() {
        CANdleConfiguration config = new CANdleConfiguration();
        
        // Configure LED settings
        LEDConfigs ledConfig = new LEDConfigs();
        ledConfig.BrightnessScalar = CANdleConstants.DEFAULT_BRIGHTNESS;
        ledConfig.StripType = StripTypeValue.GRB; // Common addressable LED type
        config.LED = ledConfig;
        
        // Configure CANdle features
        CANdleFeaturesConfigs featuresConfig = new CANdleFeaturesConfigs();
        featuresConfig.Enable5VRail = Enable5VRailValue.Enabled;
        featuresConfig.StatusLedWhenActive = StatusLedWhenActiveValue.Enabled;
        config.CANdleFeatures = featuresConfig;
        
        candle.getConfigurator().apply(config);
    }
    
    /**
     * Sets the LEDs to a solid color
     * 
     * @param color The RGBW color to set
     */
    public void setColor(RGBWColor color) {
        currentAnimation = AnimationMode.NONE;
        
        if (isSimulation) {
            simColor = color;
        } else {
            candle.setControl(
                new SolidColor(
                    CANdleConstants.LED_START_INDEX,
                    CANdleConstants.LED_END_INDEX
                ).withColor(color)
            );
        }
    }
    
    /**
     * Sets the LEDs to a solid RGB color (white component = 0)
     * 
     * @param red Red component (0-255)
     * @param green Green component (0-255)
     * @param blue Blue component (0-255)
     */
    public void setRGB(int red, int green, int blue) {
        setColor(new RGBWColor(red, green, blue));
    }
    
    /**
     * Sets the LEDs to a solid RGBW color
     * 
     * @param red Red component (0-255)
     * @param green Green component (0-255)
     * @param blue Blue component (0-255)
     * @param white White component (0-255)
     */
    public void setRGBW(int red, int green, int blue, int white) {
        setColor(new RGBWColor(red, green, blue, white));
    }
    
    /**
     * Turns off all LEDs
     */
    public void clearLEDs() {
        setColor(new RGBWColor(0, 0, 0));
    }
    
    /**
     * Clears any running animation
     */
    public void clearAnimation() {
        currentAnimation = AnimationMode.NONE;
        
        if (!isSimulation) {
            // Use EmptyAnimation to clear all animation slots
            for (int slot = 0; slot < 8; slot++) {
                candle.setControl(new EmptyAnimation(slot));
            }
        }
    }
    
    /**
     * Sets the brightness of the LEDs
     * 
     * @param brightness Brightness scalar (0.0 to 1.0)
     */
    public void setBrightness(double brightness) {
        brightness = Math.max(0.0, Math.min(1.0, brightness)); // Clamp to [0, 1]
        
        if (isSimulation) {
            simBrightness = brightness;
        } else {
            LEDConfigs ledConfig = new LEDConfigs();
            ledConfig.BrightnessScalar = brightness;
            
            CANdleConfiguration config = new CANdleConfiguration();
            config.LED = ledConfig;
            
            candle.getConfigurator().apply(config);
        }
    }
    
    /**
     * Starts a rainbow animation
     * 
     * @param brightness Brightness scalar (0.0 to 1.0)
     * @param speed Frame rate in Hz (2-1000)
     */
    public void setRainbow(double brightness, double speed) {
        currentAnimation = AnimationMode.RAINBOW;
        
        if (isSimulation) {
            simBrightness = brightness;
        } else {
            candle.setControl(
                new RainbowAnimation(
                    CANdleConstants.LED_START_INDEX,
                    CANdleConstants.LED_END_INDEX
                )
                .withBrightness(brightness)
                .withFrameRate(speed)
                .withSlot(0)
            );
        }
    }
    
    /**
     * Starts a fire animation
     * 
     * @param brightness Brightness scalar (0.0 to 1.0)
     * @param speed Frame rate in Hz (2-1000)
     * @param sparking Sparking factor (0.0 to 1.0)
     * @param cooling Cooling factor (0.0 to 1.0)
     */
    public void setFire(double brightness, double speed, double sparking, double cooling) {
        currentAnimation = AnimationMode.FIRE;
        
        if (isSimulation) {
            simBrightness = brightness;
        } else {
            candle.setControl(
                new FireAnimation(
                    CANdleConstants.LED_START_INDEX,
                    CANdleConstants.LED_END_INDEX
                )
                .withBrightness(brightness)
                .withFrameRate(speed)
                .withSparking(sparking)
                .withCooling(cooling)
                .withSlot(0)
            );
        }
    }
    
    /**
     * Starts a Larson (Cylon/KITT) animation
     * 
     * @param color The color of the animation
     * @param speed Frame rate in Hz (2-1000)
     */
    public void setLarson(RGBWColor color, double speed) {
        currentAnimation = AnimationMode.LARSON;
        
        if (isSimulation) {
            simColor = color;
        } else {
            candle.setControl(
                new LarsonAnimation(
                    CANdleConstants.LED_START_INDEX,
                    CANdleConstants.LED_END_INDEX
                )
                .withColor(color)
                .withFrameRate(speed)
                .withSlot(0)
            );
        }
    }
    
    /**
     * Starts a strobe animation
     * 
     * @param color The color of the strobe
     * @param speed Frame rate in Hz (2-1000)
     */
    public void setStrobe(RGBWColor color, double speed) {
        currentAnimation = AnimationMode.STROBE;
        
        if (isSimulation) {
            simColor = color;
        } else {
            candle.setControl(
                new StrobeAnimation(
                    CANdleConstants.LED_START_INDEX,
                    CANdleConstants.LED_END_INDEX
                )
                .withColor(color)
                .withFrameRate(speed)
                .withSlot(0)
            );
        }
    }
    
    /**
     * Starts a twinkle animation
     * 
     * @param color The color of the twinkle effect
     * @param speed Frame rate in Hz (2-1000)
     * @param maxLEDsOnProportion Maximum proportion of LEDs that can be on (0.1 to 1.0)
     */
    public void setTwinkle(RGBWColor color, double speed, double maxLEDsOnProportion) {
        currentAnimation = AnimationMode.TWINKLE;
        
        if (isSimulation) {
            simColor = color;
        } else {
            candle.setControl(
                new TwinkleAnimation(
                    CANdleConstants.LED_START_INDEX,
                    CANdleConstants.LED_END_INDEX
                )
                .withColor(color)
                .withFrameRate(speed)
                .withMaxLEDsOnProportion(maxLEDsOnProportion)
                .withSlot(0)
            );
        }
    }
    
    /**
     * Starts a color flow animation
     * 
     * @param color The color of the flow
     * @param speed Frame rate in Hz (2-1000)
     */
    public void setColorFlow(RGBWColor color, double speed) {
        currentAnimation = AnimationMode.COLOR_FLOW;
        
        if (isSimulation) {
            simColor = color;
        } else {
            candle.setControl(
                new ColorFlowAnimation(
                    CANdleConstants.LED_START_INDEX,
                    CANdleConstants.LED_END_INDEX
                )
                .withColor(color)
                .withFrameRate(speed)
                .withSlot(0)
            );
        }
    }
    
    /**
     * Sets the LEDs to the alliance color (red or blue)
     */
    public void setAllianceColor() {
        Optional<Alliance> alliance = DriverStation.getAlliance();
        
        if (alliance.isPresent()) {
            if (alliance.get() == Alliance.Red) {
                setRGB(255, 0, 0); // Red
            } else {
                setRGB(0, 0, 255); // Blue
            }
        } else {
            setRGB(0, 255, 0); // Green if alliance unknown
        }
    }
    
    /**
     * Sets LED color based on robot state
     */
    public void setRobotStateColor() {
        if (DriverStation.isDisabled()) {
            setRGB(255, 165, 0); // Orange for disabled
        } else if (DriverStation.isAutonomous()) {
            setAllianceColor(); // Alliance color in autonomous
        } else if (DriverStation.isTeleop()) {
            setRGB(0, 255, 0); // Green for teleop
        } else if (DriverStation.isTest()) {
            setRGB(255, 0, 255); // Magenta for test
        } else {
            setRGB(255, 255, 0); // Yellow for unknown/estop
        }
    }
    
    /**
     * Gets the current temperature of the CANdle in Celsius
     * 
     * @return Temperature in Celsius, or 25.0 in simulation
     */
    public double getTemperature() {
        if (isSimulation) {
            return 25.0; // Simulated temperature
        }
        
        var tempSignal = candle.getDeviceTemp();
        tempSignal.refresh();
        return tempSignal.getValueAsDouble();
    }
    
    /**
     * Gets the current draw of the CANdle in Amps
     * 
     * @return Current in Amps, or 0.5 in simulation
     */
    public double getCurrent() {
        if (isSimulation) {
            return 0.5; // Simulated current
        }
        
        var currentSignal = candle.getOutputCurrent();
        currentSignal.refresh();
        return currentSignal.getValueAsDouble();
    }
    
    /**
     * Gets the input voltage to the CANdle in Volts
     * 
     * @return Voltage in Volts, or 12.0 in simulation
     */
    public double getVoltage() {
        if (isSimulation) {
            return 12.0; // Simulated voltage
        }
        
        var voltageSignal = candle.getSupplyVoltage();
        voltageSignal.refresh();
        return voltageSignal.getValueAsDouble();
    }
    
    @Override
    public void periodic() {
        // Update telemetry
        SmartDashboard.putString("CANdle/Animation", currentAnimation.toString());
        SmartDashboard.putNumber("CANdle/Temperature", getTemperature());
        SmartDashboard.putNumber("CANdle/Current", getCurrent());
        SmartDashboard.putNumber("CANdle/Voltage", getVoltage());
        
        if (isSimulation) {
            SmartDashboard.putString("CANdle/SimColor", simColor.toHexString());
            SmartDashboard.putNumber("CANdle/SimBrightness", simBrightness);
        }
    }
    
    /**
     * Gets the current animation mode
     * 
     * @return The current AnimationMode
     */
    public AnimationMode getCurrentAnimation() {
        return currentAnimation;
    }
}
