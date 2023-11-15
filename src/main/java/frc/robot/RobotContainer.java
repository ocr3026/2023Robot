// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import frc.robot.util.OcrMath;
import frc.robot.keybinds.*;
/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
	// Globals
	public static boolean isShiftedDown = false;

	PIDController PID = new PIDController(0, 0, 0);

	// Subsystems
	public static SwerveDriveSubsystem driveSubsystem = new SwerveDriveSubsystem();
	ArmSubsystem armSubsystem = new ArmSubsystem();
	IntakeSubsystem intakeSubsystem = new IntakeSubsystem(armSubsystem);
	VisionSubsystem visionSubsystem = new VisionSubsystem();
	Wrist wristSubsystem = new Wrist();

	// Commands
	private final Command driveRobotCentric = new DriveRobotCentric(driveSubsystem);
	private final Command driveFieldCentric = new DriveFieldCentric(driveSubsystem);
	private final Command autoBalance = new AutoBalance(driveSubsystem);
	private final Command autoBalance2 = new AutoBalance(driveSubsystem);
	private final AutoBalance autoBalance4 = new AutoBalance(driveSubsystem);
	private final AutoBalance autoBalance5 = new AutoBalance(driveSubsystem);
	private final AutoBalance autoBalance3 = new AutoBalance(driveSubsystem);
	private final Command autoPlaceAndBalance =
		new AutoPlaceAndBalance(driveSubsystem, armSubsystem, intakeSubsystem, visionSubsystem, autoBalance3);
	private final Command autoPlaceAndBalance2 =
		new AutoPlaceAndBalance(driveSubsystem, armSubsystem, intakeSubsystem, visionSubsystem, autoBalance4);
	private final Command autoPlaceAndBalance3 =
		new AutoPlaceAndBalance(driveSubsystem, armSubsystem, intakeSubsystem, visionSubsystem, autoBalance5);
	/*private final Command autoPlaceAndBackwards = 
		new AutoPlaceAndBackwards(driveSubsystem, armSubsystem, intakeSubsystem, visionSubsystem, autoBalance5);*/

	private final AutoBackwards autoBackwards = new AutoBackwards(driveSubsystem);
	// Drivetrain Triggers
	private final Trigger fieldTrigger = Constants.translationJoystick.button(2);
	private final Trigger balanceTrigger = Constants.translationJoystick.button(3);


	// Claw Triggers
	

	// private final Trigger autoPlaceTrigger = Constants.xboxController.a();
	// private final Trigger tuckInTrigger = Constants.xboxController.x();
	// private final Trigger armFlatTrigger = Constants.xboxController.y();

	Compressor compressor = new Compressor(1, PneumaticsModuleType.REVPH);
	public SendableChooser<Command> sendableChooser = new SendableChooser<>();

	//
	/**
	 * The container for the robot. Contains subsystems, OI devices, and commands.
	 */
	public RobotContainer() {
		// Configure the trigger bindings
		configureBindings();
		compressor.enableDigital();
		//  Configure the default commands
		DriverDefaultControls.fieldTrigger.toggleOnTrue(driveRobotCentric);
		sendableChooser.addOption(
			"AutoBalance", autoPlaceAndBalance2.andThen (
				new FunctionalCommand(
					()
						-> {},
					()
						-> driveSubsystem.driveRobotCentric(-.4, 0, 0),
					t -> {}, () -> !OcrMath.isWithin(Constants.gyro.getPitch(), 13))
					.andThen(new FunctionalCommand(
						()
							-> {},
						()
							-> driveSubsystem.driveRobotCentric(-.75, 0, 0),
						t -> {}, () -> !OcrMath.isWithin(Constants.gyro.getPitch(), 18)))
					.andThen(autoBalance2)));
		sendableChooser.addOption("DriveBack", autoPlaceAndBalance3.andThen(autoBackwards));
		sendableChooser.setDefaultOption("nothing", new RunCommand(() -> {}));
		//sendableChooser.addOption("Place", autoPlaceAndBalance);
		SmartDashboard.putData(sendableChooser);
	}

	private void configureBindings() {
		//sendableChooser.addOption("DriveBack", autoBackwards);

		ManipulatorDefaultControls.clawIntakTrigger.whileTrue(intakeSubsystem.ClawIntake());
		ManipulatorDefaultControls.clawOutakeTrigger.whileTrue(intakeSubsystem.ClawOutake());
		
		fieldTrigger.toggleOnTrue(driveRobotCentric);

		

		intakeSubsystem.setDefaultCommand(armSubsystem.armCommand(intakeSubsystem));

		visionSubsystem.setDefaultCommand((armSubsystem.clawCommand(visionSubsystem)));
		ManipulatorDefaultControls.extendArm.whileTrue(armSubsystem.ExtendArm());
		ManipulatorDefaultControls.retractArm.whileTrue(armSubsystem.ReturnArm());

		// autoPlaceTrigger.whileTrue(autoPlace);

		DriverDefaultControls.balanceTrigger.whileTrue(autoBalance);
		driveSubsystem.setDefaultCommand(driveFieldCentric);

	//	Constants.xboxController.rightBumper().onTrue(autoPlaceAndBalance);
	//	Constants.xboxController.rightBumper().onFalse(new InstantCommand(() ->  CommandScheduler.getInstance().cancel(autoPlaceAndBalance)));
		DriverDefaultControls.resetGyro.onTrue(
			new InstantCommand(() -> Constants.gyro.reset()));
		DriverDefaultControls.zeroClaw.onTrue(
			new InstantCommand(() -> armSubsystem.clawEncoder.setPosition(0)));
		DriverDefaultControls.zeroArm.onTrue(
			new InstantCommand(() -> ArmSubsystem.armEncoder.setPosition(0)));

		new Trigger(DriverStation::isDisabled).onTrue(driveSubsystem.OnDisableCommand());

		new Trigger(DriverStation::isEnabled)
			.onTrue(new InstantCommand(() -> Constants.gyro.setYaw(180)));
		new Trigger(DriverStation::isEnabled).onTrue(new InstantCommand(() -> driveSubsystem.zeroWheel()));
		DriverDefaultControls.zeroWheel.onTrue(driveSubsystem.zeroWheel());

	//	Constants.translationJoystick.button(1)
		//	.onTrue(new InstantCommand(() -> driveSubsystem.setCoast()))
		//	.onFalse(new InstantCommand(() -> driveSubsystem.setCoast()));

		Constants.xboxController.button(7).toggleOnTrue(new InstantCommand(() -> driveSubsystem.setCoast()));

		Constants.xboxController.button(8).toggleOnTrue(new InstantCommand(() -> driveSubsystem.setBrake()));

		DriverDefaultControls.slowerDrive
			.onTrue(new InstantCommand(() -> isShiftedDown = true))
			.onFalse(new InstantCommand(() -> isShiftedDown = false));


		

		// Constants.xboxController.rightBumper().onTrue(resetMax);
	}
	public Command getAutonomousCommand() { return sendableChooser.getSelected(); }
}