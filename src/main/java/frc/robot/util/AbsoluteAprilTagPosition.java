package frc.robot.util;

import frc.robot.subsystems.*;

public class AbsoluteAprilTagPosition {
    VisionSubsystem visionSubsystem;
    SwerveDriveSubsystem swerveDriveSubsystem;
    public AbsoluteAprilTagPosition (SwerveDriveSubsystem subsystem, VisionSubsystem subsystem2) {
        swerveDriveSubsystem = subsystem;
        visionSubsystem = subsystem2;
    }
    public void moveToSpot (double xPos, double yPos) {
        
    }
}
