package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveDriveSubsystem;

public class AutoBackwards extends CommandBase {
	Timer timer = new Timer();
	SwerveDriveSubsystem swerveDriveSubsystem;

	public AutoBackwards(SwerveDriveSubsystem subsystem) {
		addRequirements(subsystem);
		swerveDriveSubsystem = subsystem;
	}
	@Override
	public void initialize() {
		timer.reset();
		timer.start();
	}
	@Override
	public void execute() {
		if (!timer.hasElapsed(.8)) {
			swerveDriveSubsystem.driveRobotCentric(-0.25, 0, 0);
		} else {
			swerveDriveSubsystem.driveRobotCentric(0, 0, 0);
		}
	}

	@Override
	public boolean isFinished() {
		return timer.hasElapsed(1.1);
	}
}
