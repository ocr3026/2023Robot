package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveDriveSubsystem;
import frc.robot.util.OcrMath;

public class AutoBalance extends CommandBase {
	private SwerveDriveSubsystem driveSubsystem;
	PIDController balancePID = new PIDController(.0095/* .01295*/, 0, 0.0014001);
	double prevSpeed = 0;
	double reductionMulitplier = 1;
	int times = 0;

	public AutoBalance(SwerveDriveSubsystem subsystem) {
		SmartDashboard.putNumber("P", balancePID.getP());
		SmartDashboard.putNumber("I", balancePID.getI());
		SmartDashboard.putNumber("D", balancePID.getD());
		addRequirements(subsystem);
		driveSubsystem = subsystem;
	}

	@Override
	public void initialize() {
		prevSpeed = 0;
		reductionMulitplier = 1;
		times = 0;
		balancePID =
			new PIDController(SmartDashboard.getNumber("P", 0), SmartDashboard.getNumber("I", 0),
		                      SmartDashboard.getNumber("D", 0));
	}

	public void autoBalance() {
		switch (Constants.autoBalanceStage) {
		case none:
			break;

		case driveBackwards:
			if (!OcrMath.isWithin(Constants.gyro.getPitch(), 13)) {
				driveSubsystem.driveRobotCentric(-.3, 0, 0);
			} else {
				Constants.autoBalanceStage = Constants.autoBalance.gyroSmaller;
			}
			break;

		case gyroSmaller:
			if (!OcrMath.isWithin(Constants.gyro.getPitch(), 16)) {
				driveSubsystem.driveRobotCentric(-.75, 0, 0);
			}
		}
	}

	@Override
	public void execute() {
		double currentSpeed = balancePID.calculate(Constants.gyro.getPitch(), 0);

		// first time through only?
		if (prevSpeed == 0) {
			prevSpeed = currentSpeed;
		}

		// if direction changed, reduce speed
		if ((prevSpeed > 0 && currentSpeed < 0) || (prevSpeed < 0 && currentSpeed > 0)) {
			reductionMulitplier *= .93;
		}

		if (OcrMath.isWithin(Constants.gyro.getPitch(), 176)) {

			driveSubsystem.driveRobotCentric(-currentSpeed * reductionMulitplier, 0, 0);

		} else {
			driveSubsystem.driveRobotCentric(0, 0, 0);
			times = times + 1;
		}

		prevSpeed = currentSpeed;
	}

	@Override
	public boolean isFinished() {
		return times == 5 && OcrMath.isWithin(Constants.gyro.getPitch(), 3);
	}

	@Override
	public void end(boolean interupt) {}
}
