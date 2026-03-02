# TODO: Implement Vision Pose Instead of Wheel Pose

## Plan:
1. [x] Modify VisionSubsystem.java - Add method to get most trusted vision measurement with quality metrics
2. [x] Modify CommandSwerveDrivetrain.java - Add getPose() that returns vision pose when available
3. [x] Modify RobotContainer.java - Connect VisionSubsystem to CommandSwerveDrivetrain

## Changes:
- VisionSubsystem: Add VisionMeasurement record with getTrustScore() method
- VisionSubsystem: Add getMostTrustedVisionMeasurement() method
- VisionSubsystem: Add hasValidVisionMeasurement() method
- CommandSwerveDrivetrain: Add setVisionSubsystem() method to connect vision
- CommandSwerveDrivetrain: Add getPose() method that returns vision pose when available
- RobotContainer: Call drivetrain.setVisionSubsystem(visionSubsystem) in constructor
