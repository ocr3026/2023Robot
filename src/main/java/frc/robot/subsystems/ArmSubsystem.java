package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.OcrMath;
import java.util.LinkedList;
import java.util.Queue;


public class ArmSubsystem extends SubsystemBase {
	Queue<Double> averageCurrent = new LinkedList<Double>();

	double currentPos;
	// Motor Controllers
	public static final CANSparkMax armMotor = new CANSparkMax(20, MotorType.kBrushless);
	public final CANSparkMax clawMotor = new CANSparkMax(21, MotorType.kBrushless);

	// Solenoids
	public DoubleSolenoid armSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, 2, 3);

	// Encoders
	//arm Encoder Negative
	public static final RelativeEncoder armEncoder = armMotor.getEncoder();
	//claw Encoder Positive
	public RelativeEncoder clawEncoder = clawMotor.getEncoder();

	// PID
	PIDController clawPID = new PIDController(.135, 0, 0);
	PIDController armPID = new PIDController(.015, 0, 0);

	// Constants
	//arm Encoder Negative is negative
	final int armLowestValue = -3;
	final int armHighestValue = -270;

	public ArmSubsystem() {
		armMotor.setIdleMode(IdleMode.kBrake);
		clawMotor.setIdleMode(IdleMode.kBrake);
		currentPos = 0;
	}

	public void LiftArm(double speed) {

		//slow down Arm whenn it  is  high
		if (!OcrMath.isWithin(armEncoder.getPosition(), 240)) {
			if (speed >= 0) {
				armMotor.set(speed);
			} else {
				armMotor.set(speed * .5);
			}
		}
		// Soft Limit: arm cant go in if  claw is up
		if (armEncoder.getPosition() <= armHighestValue)  {
			if (speed >= 0) {
				armMotor.set(speed);
			} else {
				armMotor.set(0);
			}
			//lower soft limit
		} else if (armEncoder.getPosition() >= armLowestValue &&  clawEncoder.getPosition() > 12) {
			if (speed <= 0) {
				armMotor.set(speed);
			} else {
				armMotor.set(0);
			}
		}
		//lower hard switch  limit and zero
		else if (!Constants.limit.get()) {
			armEncoder.setPosition(0);
			clawEncoder.setPosition(0);
			if (speed <= 0) {
				armMotor.set(speed);
			} else {
				armMotor.set(0);
			}
		}
		else if (Constants.upperSwitch.get()) {
			if (speed >= 0) {
				armMotor.set(speed);
			} else {
				armMotor.set(0);
			}
		} else if (Constants.lowerSwitch.get()) {
			if (speed <= 0) {
				armMotor.set(speed);
			} else {
				armMotor.set(0);
			}
		}
		else {
			armMotor.set(speed);
		}
	}
	public Command armCommand(Subsystem subsystem) {
		return new RunCommand(
			() -> LiftArm(OcrMath.deadband(Constants.xboxController.getLeftY(), .1)), subsystem);
	}

	public Command ExtendArm() { return new RunCommand(() -> armSolenoid.set(Value.kReverse)); }
	public Command ReturnArm() { return new RunCommand(() -> armSolenoid.set(Value.kForward)); }

	public void ClawRotation(double speed) {

		if (armEncoder.getPosition() >= -11 && clawEncoder.getPosition() >= 10) {
			if (speed > 0) {
				clawMotor.set(0);
			} else {
				clawMotor.set(-speed);
			}
		}
		else if(clawEncoder.getPosition() > 72) {
			if (speed < 0) {
				clawMotor.set(0);
			} else {
				clawMotor.set(-speed);
			}
		}
		 else {
			clawMotor.set(-speed);
		}
	}

	public Command clawCommand(Subsystem subsystem) {
		return new RunCommand(
			()
				-> ClawRotation(OcrMath.deadband(Constants.xboxController.getRightY(), .1) * .25),
			subsystem);
	}

	/* 	public void ClawAutoBalance() {
	        ClawRotation(clawPID.calculate(clawEncoder.getPosition(), armEncoder.getPosition()));
	    }
	    public void AutoPositionManipulator(double armDegree, double clawDegree) {
	        LiftArm(armPID.calculate(armEncoder.getPosition(), armDegree));
	        ClawRotation(-clawPID.calculate(clawEncoder.getPosition(), clawDegree));
	    }*/

	@Override
	public void periodic() {
		SmartDashboard.putNumber("Amps", armMotor.getOutputCurrent());
		SmartDashboard.putNumber("wristEncoder", clawEncoder.getPosition());
		SmartDashboard.putNumber("armEncoder", armEncoder.getPosition());

		if (clawMotor.getOutputCurrent() > 18) {
			clawMotor.disable();
			System.out.println("Claw motor overcurrent");
		}
		/*if((armMotor.getOutputCurrent() > 19)) {
		    System.out.println("Arm motor overcurrent");
		    Constants.killMotor = true;
		    armMotor.disable();
		}*/

		averageCurrent.add(armMotor.getOutputCurrent());
		if (averageCurrent.size() > 100) {
			averageCurrent.remove();
		}
		for (double d : averageCurrent) {
			Constants.total += d;
		}
		
		//System.out.println(Constants.total);
	}
}