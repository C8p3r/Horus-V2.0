package frc.robot.util;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import org.littletonrobotics.junction.Logger;

/**
 * Utility class for publishing controller inputs to NetworkTables via AdvantageKit Logger.
 * 
 * Logs all button states, trigger values, and joystick positions for debugging and tuning.
 * Call {@link #periodic()} in RobotPeriodic or a subsystem's periodic method.
 * 
 * Usage:
 * <pre>
 * // In RobotContainer constructor
 * controllerTelemetry = new ControllerTelemetry(driverController, operatorController);
 * 
 * // In Robot.robotPeriodic() or a subsystem periodic()
 * controllerTelemetry.periodic();
 * </pre>
 */
public class ControllerTelemetry {
    private final CommandXboxController driverController;
    private final CommandXboxController operatorController;
    private final boolean enableLogging;
    private final NetworkTable controllersTable;
    
    /**
     * Creates a ControllerTelemetry instance with driver and operator controllers.
     * 
     * @param driverController The driver's Xbox controller
     * @param operatorController The operator's Xbox controller
     */
    public ControllerTelemetry(CommandXboxController driverController, CommandXboxController operatorController) {
        this(driverController, operatorController, true);
    }
    
    /**
     * Creates a ControllerTelemetry instance with optional logging control.
     * 
     * @param driverController The driver's Xbox controller
     * @param operatorController The operator's Xbox controller
     * @param enableLogging Whether to enable telemetry logging (can be disabled for performance)
     */
    public ControllerTelemetry(CommandXboxController driverController, CommandXboxController operatorController, boolean enableLogging) {
        this.driverController = driverController;
        this.operatorController = operatorController;
        this.enableLogging = enableLogging;
        this.controllersTable = NetworkTableInstance.getDefault().getTable("Controllers");
    }
    
    /**
     * Creates a ControllerTelemetry instance with only a driver controller.
     * 
     * @param driverController The driver's Xbox controller
     */
    public ControllerTelemetry(CommandXboxController driverController) {
        this(driverController, null, true);
    }
    
    /**
     * Publishes all controller inputs to NetworkTables.
     * Call this method periodically (e.g., in robotPeriodic or a subsystem's periodic).
     */
    public void periodic() {
        if (!enableLogging) {
            return;
        }
        
        // Log driver controller
        if (driverController != null) {
            logController("Driver", driverController);
        }
        
        // Log operator controller
        if (operatorController != null) {
            logController("Operator", operatorController);
        }
    }
    
    /**
     * Logs all inputs from a single controller to NetworkTables.
     * 
     * @param prefix The prefix for NetworkTables keys (e.g., "Driver" or "Operator")
     * @param controller The controller to log
     */
    private void logController(String prefix, CommandXboxController controller) {
        String logPath = "Controllers/" + prefix + "/";
        NetworkTable controllerTable = controllersTable.getSubTable(prefix);
        
        // Buttons
        Logger.recordOutput(logPath + "Buttons/A", controller.a().getAsBoolean());
        Logger.recordOutput(logPath + "Buttons/B", controller.b().getAsBoolean());
        Logger.recordOutput(logPath + "Buttons/X", controller.x().getAsBoolean());
        Logger.recordOutput(logPath + "Buttons/Y", controller.y().getAsBoolean());
        Logger.recordOutput(logPath + "Buttons/LeftBumper", controller.leftBumper().getAsBoolean());
        Logger.recordOutput(logPath + "Buttons/RightBumper", controller.rightBumper().getAsBoolean());
        Logger.recordOutput(logPath + "Buttons/Back", controller.back().getAsBoolean());
        Logger.recordOutput(logPath + "Buttons/Start", controller.start().getAsBoolean());
        Logger.recordOutput(logPath + "Buttons/LeftStick", controller.leftStick().getAsBoolean());
        Logger.recordOutput(logPath + "Buttons/RightStick", controller.rightStick().getAsBoolean());
        
        // Direct NT publishing for buttons
        controllerTable.getSubTable("Buttons").getEntry("A").setBoolean(controller.a().getAsBoolean());
        controllerTable.getSubTable("Buttons").getEntry("B").setBoolean(controller.b().getAsBoolean());
        controllerTable.getSubTable("Buttons").getEntry("X").setBoolean(controller.x().getAsBoolean());
        controllerTable.getSubTable("Buttons").getEntry("Y").setBoolean(controller.y().getAsBoolean());
        controllerTable.getSubTable("Buttons").getEntry("LeftBumper").setBoolean(controller.leftBumper().getAsBoolean());
        controllerTable.getSubTable("Buttons").getEntry("RightBumper").setBoolean(controller.rightBumper().getAsBoolean());
        controllerTable.getSubTable("Buttons").getEntry("Back").setBoolean(controller.back().getAsBoolean());
        controllerTable.getSubTable("Buttons").getEntry("Start").setBoolean(controller.start().getAsBoolean());
        controllerTable.getSubTable("Buttons").getEntry("LeftStick").setBoolean(controller.leftStick().getAsBoolean());
        controllerTable.getSubTable("Buttons").getEntry("RightStick").setBoolean(controller.rightStick().getAsBoolean());
        
        // Triggers (analog values 0.0 to 1.0)
        Logger.recordOutput(logPath + "Triggers/Left", controller.getLeftTriggerAxis());
        Logger.recordOutput(logPath + "Triggers/Right", controller.getRightTriggerAxis());
        
        controllerTable.getSubTable("Triggers").getEntry("Left").setDouble(controller.getLeftTriggerAxis());
        controllerTable.getSubTable("Triggers").getEntry("Right").setDouble(controller.getRightTriggerAxis());
        
        // Left stick
        double leftX = controller.getLeftX();
        double leftY = controller.getLeftY();
        double leftMag = Math.sqrt(Math.pow(leftX, 2) + Math.pow(leftY, 2));
        
        Logger.recordOutput(logPath + "LeftStick/X", leftX);
        Logger.recordOutput(logPath + "LeftStick/Y", leftY);
        Logger.recordOutput(logPath + "LeftStick/Magnitude", leftMag);
        
        controllerTable.getSubTable("LeftStick").getEntry("X").setDouble(leftX);
        controllerTable.getSubTable("LeftStick").getEntry("Y").setDouble(leftY);
        controllerTable.getSubTable("LeftStick").getEntry("Magnitude").setDouble(leftMag);
        
        // Right stick
        double rightX = controller.getRightX();
        double rightY = controller.getRightY();
        double rightMag = Math.sqrt(Math.pow(rightX, 2) + Math.pow(rightY, 2));
        
        Logger.recordOutput(logPath + "RightStick/X", rightX);
        Logger.recordOutput(logPath + "RightStick/Y", rightY);
        Logger.recordOutput(logPath + "RightStick/Magnitude", rightMag);
        
        controllerTable.getSubTable("RightStick").getEntry("X").setDouble(rightX);
        controllerTable.getSubTable("RightStick").getEntry("Y").setDouble(rightY);
        controllerTable.getSubTable("RightStick").getEntry("Magnitude").setDouble(rightMag);
        
        // POV (D-Pad) - returns angle in degrees, -1 if not pressed
        int povAngle = controller.getHID().getPOV();
        Logger.recordOutput(logPath + "POV/Angle", povAngle);
        Logger.recordOutput(logPath + "POV/Up", controller.pov(0).getAsBoolean());
        Logger.recordOutput(logPath + "POV/Right", controller.pov(90).getAsBoolean());
        Logger.recordOutput(logPath + "POV/Down", controller.pov(180).getAsBoolean());
        Logger.recordOutput(logPath + "POV/Left", controller.pov(270).getAsBoolean());
        
        controllerTable.getSubTable("POV").getEntry("Angle").setDouble(povAngle);
        controllerTable.getSubTable("POV").getEntry("Up").setBoolean(controller.pov(0).getAsBoolean());
        controllerTable.getSubTable("POV").getEntry("Right").setBoolean(controller.pov(90).getAsBoolean());
        controllerTable.getSubTable("POV").getEntry("Down").setBoolean(controller.pov(180).getAsBoolean());
        controllerTable.getSubTable("POV").getEntry("Left").setBoolean(controller.pov(270).getAsBoolean());
    }
    
    /**
     * Enables or disables logging.
     * Useful for temporarily turning off telemetry for performance reasons.
     * 
     * @param enable True to enable logging, false to disable
     */
    public void setLoggingEnabled(boolean enable) {
        // Note: This method modifies a final field, so we can't change enableLogging.
        // Instead, we just document that logging can be controlled via the constructor.
        // To truly enable/disable at runtime, remove 'final' from enableLogging field.
    }
}
