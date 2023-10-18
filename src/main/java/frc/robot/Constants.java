// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.util.AHRSMixin;

import org.photonvision.PhotonCamera;


/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
	// Motor Constant
	public static final double maxLeadscrewRotations = 18.75;
	// Controllers
	public static final CommandJoystick translationJoystick = new CommandJoystick(0);
	public static final CommandJoystick rotationJoystick = new CommandJoystick(1);
	public static final CommandXboxController xboxController = new CommandXboxController(2);
	public static double x = 0;
	public static double total = 0;

	public static Boolean killMotor = false;

	public static final double deadband = 0.15;
	public static final double distFromConeIn = 19;
	public static final double distFromConeBackIn = 54;


	public static DigitalInput upperSwitch = new DigitalInput(5);
	public static DigitalInput lowerSwitch = new DigitalInput(7);

	// Gyro
	public static final AHRSMixin gyro = new AHRSMixin(SerialPort.Port.kMXP);

	// Swerve Module Positions
	public static final Translation2d frontLeftModulePos = new Translation2d(-14.25, -14.25);
	public static final Translation2d rearLeftModulePos = new Translation2d(-14.25, 14.25);
	public static final Translation2d frontRightModulePos = new Translation2d(14.25, -14.25);
	public static final Translation2d rearRightModulePos = new Translation2d(14.25, 14.25);

	// Vision
	public static final PhotonCamera camera = new PhotonCamera("USB_webcam");
	public static final double visionTolerance = 0.25;

	public enum PowerGridPositions {
		Blue1(new Pose2d(0, 0, Rotation2d.fromDegrees(180))),
		Blue2(new Pose2d(0, 0, Rotation2d.fromDegrees(180))),
		Blue3(new Pose2d(0, 0, Rotation2d.fromDegrees(180))),
		Blue4(new Pose2d(0, 0, Rotation2d.fromDegrees(180))),
		Blue5(new Pose2d(0, 0, Rotation2d.fromDegrees(180))),
		Blue6(new Pose2d(0, 0, Rotation2d.fromDegrees(180))),
		Blue7(new Pose2d(0, 0, Rotation2d.fromDegrees(180))),
		Blue8(new Pose2d(0, 0, Rotation2d.fromDegrees(180))),
		Blue9(new Pose2d(0, 0, Rotation2d.fromDegrees(180))),
		Red1(new Pose2d(0, 0, Rotation2d.fromDegrees(0))),
		Red2(new Pose2d(0, 0, Rotation2d.fromDegrees(0))),
		Red3(new Pose2d(0, 0, Rotation2d.fromDegrees(0))),
		Red4(new Pose2d(0, 0, Rotation2d.fromDegrees(0))),
		Red5(new Pose2d(0, 0, Rotation2d.fromDegrees(0))),
		Red6(new Pose2d(0, 0, Rotation2d.fromDegrees(0))),
		Red7(new Pose2d(0, 0, Rotation2d.fromDegrees(0))),
		Red8(new Pose2d(0, 0, Rotation2d.fromDegrees(0))),
		Red9(new Pose2d(0, 0, Rotation2d.fromDegrees(0)));

		public final Pose2d pose;

		private PowerGridPositions(Pose2d pose) { this.pose = pose; }
	}

	// Shift Down
	public static final double driveFactor = 0.5;
	public static DigitalInput limit = new DigitalInput(8);

	public enum eAutoPlace { none, zeroWheels, driveBackwards, raiseArm, extendArm, zeroWheel2,zeroWheel3, driveForward, moveWrist, retractArm, ejectPiece, stopOutake, driveAway, lowerArm, lowerArmToBalance, driveToBalance, gyroSmaller, autoBalance };
	public static eAutoPlace autoPlaceStage = eAutoPlace.none;


	public enum autoBalance {
		none,
		driveBackwards,
		gyroSmaller,
		balance
	}
	public static autoBalance autoBalanceStage = autoBalance.none;
	public enum twoGamePieceEnum {
		none,
		raiseArm,
		extendArm,
		placePiece,
		driveToGamepiece,
		pickupPiece,
		driveToGrid
	}
	public static twoGamePieceEnum eTwoGamePiece = twoGamePieceEnum.none;

	public enum twoPieceBalanceEnum {
		none,
		raiseArm,
		extendArm,
		placePiece,
		driveToGamepiece,
		pickupPiece,
		driveToGrid,
		driveToBalance,
		autoBalance
	}
	public static twoPieceBalanceEnum eTwoPieceBalance = twoPieceBalanceEnum.none;
}
