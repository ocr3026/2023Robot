package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
	ArmSubsystem armSubsystem;
	public CANSparkMax intakeMotor1 = new CANSparkMax(23, MotorType.kBrushless);
	public CANSparkMax intakeMotor2 = new CANSparkMax(22, MotorType.kBrushless);
	public MotorControllerGroup intake = new MotorControllerGroup(intakeMotor2, intakeMotor1);

	public IntakeSubsystem(ArmSubsystem subsystem) {
		intakeMotor2.setIdleMode(IdleMode.kBrake);
		intakeMotor1.setIdleMode(IdleMode.kBrake);
		armSubsystem = subsystem;
	}

	public Command ClawIntake() {
		
		return new StartEndCommand(() -> 
		ClawIntakeFunction(), 
		() -> 
		intake.set(0));
		
		
	}
 	public void ClawIntakeFunction() {
		
		 if (armSubsystem.clawEncoder.getPosition() < 71) {
			intake.set(.4);
		 }
		 else {
			intake.set(0);
		 }
		
	}
	public void ClawOutakeFunction() {
		
		if (armSubsystem.clawEncoder.getPosition() < 71) {
		   intake.set(-.51);
		}
		else {
			intake.set(0);
		}
	   
   }

	public Command ClawOutake() {
		return new StartEndCommand(() -> ClawOutakeFunction(), () -> intake.set(0));
	}
}
