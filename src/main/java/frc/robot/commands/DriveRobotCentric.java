package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.SwerveDriveSubsystem;
import frc.robot.util.OcrMath;

public class DriveRobotCentric extends CommandBase {
	private SwerveDriveSubsystem driveSubsystem;

	public DriveRobotCentric(SwerveDriveSubsystem subsystem) {
		addRequirements(subsystem);
		driveSubsystem = subsystem;
	}

	@Override
	public void execute() {
		if (!RobotContainer.isShiftedDown) {
			driveSubsystem.driveRobotCentric(
				OcrMath.deadband(-Constants.translationJoystick.getY(), Constants.deadband),
				OcrMath.deadband(-Constants.translationJoystick.getX(), Constants.deadband),
				OcrMath.deadband(-Constants.rotationJoystick.getX(), Constants.deadband));
		} else {
			driveSubsystem.driveRobotCentric(
				OcrMath.deadband(-Constants.translationJoystick.getY(), Constants.deadband) *
					Constants.driveFactor,
				OcrMath.deadband(-Constants.translationJoystick.getX(), Constants.deadband) *
					Constants.driveFactor,
				OcrMath.deadband(-Constants.rotationJoystick.getX(), Constants.deadband) *
					Constants.driveFactor);
		}
	}

	@Override
	public boolean isFinished() {
		return false;
	}
}
