package frc.robot.subsystems;

import java.util.Optional;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.util.RollingAverage;

public class VisionSubsystem extends SubsystemBase {
	NetworkTable limelight = NetworkTableInstance.getDefault().getTable("limelight");

	PIDController translationPID = new PIDController(0, 0, 0);
	PIDController rotationPID = new PIDController(0, 0, 0);

	RollingAverage averageYaw = new RollingAverage(50);

	public Optional<Pose3d> getRobotPose() {
		if(limelight.getEntry("tv").getNumber(0).intValue() == 0) {
			return Optional.empty();
		}

		double[] data = limelight.getEntry("botpose").getDoubleArray(new double[6]);

		return Optional.of(new Pose3d(data[0], data[1], data[2], new Rotation3d(Units.degreesToRadians(data[3]), Units.degreesToRadians(data[4]), Units.degreesToRadians(data[5]))));
	}

	public Optional<Pose3d> getTeamSidePose() {
		if(limelight.getEntry("tv").getNumber(0).intValue() == 0) {
			return Optional.empty();
		}

		double[] data;

		switch(DriverStation.getAlliance()) {
			case Red:
				data = limelight.getEntry("botpose_wpired").getDoubleArray(new double[6]);
				data[1] = -data[1] + 8.014;
				break;
			case Blue:
				data = limelight.getEntry("botpose_wpiblue").getDoubleArray(new double[6]);
				break;
			default:
				return Optional.empty();
		}
		
		return Optional.of(new Pose3d(data[0], data[1], data[2], new Rotation3d(Units.degreesToRadians(data[3]), Units.degreesToRadians(data[4]), Units.degreesToRadians(data[5]))));
	}

	public Command moveToPose2DCommand(Pose2d pose) {
		Pose2d current;

		try {
			current = getTeamSidePose().get().toPose2d();
		} catch(Exception e) {
			return new InstantCommand(() -> { RobotContainer.driveSubsystem.driveFieldCentric(0, 0, 0); }, RobotContainer.driveSubsystem);
		}

		return new FunctionalCommand(null, () -> {
			RobotContainer.driveSubsystem.driveFieldCentric(translationPID.calculate(current.getX(), pose.getX()), translationPID.calculate(current.getY(), pose.getY()), 0);
		}, (Boolean interrupted) -> {
			RobotContainer.driveSubsystem.driveFieldCentric(0, 0, 0);
		}, () -> {
			if(Math.sqrt(Math.pow(current.getX() - pose.getX(), 2) + Math.sqrt(Math.pow(current.getY() - pose.getY(), 2))) < Constants.visionTolerance) {
				return true;
			}

			return false;
		}, RobotContainer.driveSubsystem);
	}
}
