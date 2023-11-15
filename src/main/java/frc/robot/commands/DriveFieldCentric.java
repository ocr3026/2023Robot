package frc.robot.commands;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.keybinds.DriverDefaultControls;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.SwerveDriveSubsystem;
import frc.robot.util.OcrMath;

public class DriveFieldCentric extends CommandBase {
	private SwerveDriveSubsystem driveSubsystem;

	public DriveFieldCentric(SwerveDriveSubsystem subsystem) {
		addRequirements(subsystem);
		driveSubsystem = subsystem;
	}

	@Override
	public void execute() {
		if (!RobotContainer.isShiftedDown) {
			driveSubsystem.driveFieldCentric(
				OcrMath.deadband(-DriverDefaultControls.driveY, Constants.deadband),
				OcrMath.deadband(-DriverDefaultControls.driveX, Constants.deadband),
				OcrMath.deadband(-Constants.rotationJoystick.getX(), Constants.deadband));
		} else {
			driveSubsystem.driveFieldCentric(
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
