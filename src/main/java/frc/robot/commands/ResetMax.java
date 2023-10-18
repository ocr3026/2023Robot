package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;

public class ResetMax extends CommandBase {
	@Override
	public void execute() {
		Constants.x = 0;
		Constants.killMotor = false;
	}

	@Override
	public boolean isFinished() {
		return true;
	}
}
