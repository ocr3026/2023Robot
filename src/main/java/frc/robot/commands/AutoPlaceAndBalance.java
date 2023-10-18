package frc.robot.commands;

import edu.wpi.first.cscore.VideoSource.ConnectionStrategy;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Constants.autoBalance;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.SwerveDriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.util.OcrMath;


public class AutoPlaceAndBalance extends CommandBase {
	Timer timer = new Timer();
	SwerveDriveSubsystem swerveDriveSubsystem;
	ArmSubsystem armSubsystem;
	IntakeSubsystem intakeSubsystem;
    VisionSubsystem visionSubsystem;
    AutoBalance autoBalance;

    double gyroPitch = 0;

	PIDController autoBalancePID = new PIDController(.007, 0, 0.0000001);

    public AutoPlaceAndBalance(SwerveDriveSubsystem subsystem, ArmSubsystem arm, IntakeSubsystem intake, VisionSubsystem vision, AutoBalance autoBala) {
        addRequirements(subsystem);
        swerveDriveSubsystem = subsystem;
        addRequirements(arm);
        armSubsystem = arm;
        addRequirements(intake);
        intakeSubsystem = intake;
        addRequirements(vision);
        visionSubsystem = vision;
        autoBalance = autoBala;
        SmartDashboard.putString("Hint", "Bad");
    }
   


    @Override 
    public void initialize() {
        swerveDriveSubsystem.zeroWheelFunc();
        Constants.autoPlaceStage = Constants.eAutoPlace.zeroWheels;
        
        timer.reset();
        timer.start();
    }

    public void redAuto() {
        
        visionSubsystem.moveToPose2DCommand(new Pose2d());
    }

    public void autoPlace() {

        switch(Constants.autoPlaceStage) {
          
            case none:
                break;

            case zeroWheels:
                swerveDriveSubsystem.zeroWheelFunc();
				armSubsystem.clawEncoder.setPosition(0);
				armSubsystem.armEncoder.setPosition(0);
                Constants.autoPlaceStage = Constants.eAutoPlace.driveBackwards;
break;

            case driveBackwards:

            if(Math.abs(swerveDriveSubsystem.frontLeftModule.driveEncoder.getPosition()) <= 8) {
                swerveDriveSubsystem.driveRobotCentric(-0.2, 0, 0);
				armSubsystem.ClawRotation(.1);
            }
            else {
                swerveDriveSubsystem.driveRobotCentric(0, 0, 0);
                Constants.autoPlaceStage = Constants.eAutoPlace.raiseArm;
            }
            break;
            case raiseArm:
                if(armSubsystem.armEncoder.getPosition() > -266) 
                {
                    armSubsystem.LiftArm(-1);
                    //System.out.println(armSubsystem.armMotor.getOutputCurrent());
                 
                    if(!(armSubsystem.armEncoder.getPosition() > -90)) {
						
                        	armSubsystem.ClawRotation(-0.17);
					
						
							
					
                    }
                    else if(armSubsystem.armEncoder.getPosition() > -90) {
                        armSubsystem.ClawRotation(0.1);
                    }
                
                }                    
                else 
                {
                   SmartDashboard.putString("Hint", "Good Job");
                   armSubsystem.LiftArm(0);
                   Constants.autoPlaceStage = Constants.eAutoPlace.extendArm;
                   
                   // Constants.autoPlaceStage = Constants.eAutoPlace.none;
                }
                break;
            case extendArm:
                SmartDashboard.putString("Hint", "great Job");
                armSubsystem.armSolenoid.set(Value.kReverse);
                timer.reset();
                if(!timer.hasElapsed(0.3)) {
                    timer.reset();
                    swerveDriveSubsystem.zeroWheelFunc();
                    Constants.autoPlaceStage = Constants.eAutoPlace.zeroWheel2;
                }
                
                break;

            case zeroWheel2:
                swerveDriveSubsystem.zeroWheelFunc();
                Constants.autoPlaceStage = Constants.eAutoPlace.driveForward;
                break;

            case driveForward:
            if (Math.abs(swerveDriveSubsystem.frontLeftModule.driveEncoder.getPosition()) <= 6.34) {
                swerveDriveSubsystem.driveRobotCentric(.2 , 0 , 0);
                intakeSubsystem.intake.set(0.4);
            }
            else {
                swerveDriveSubsystem.driveRobotCentric(0, 0, 0);
                intakeSubsystem.intake.set(0);
                Constants.autoPlaceStage = Constants.eAutoPlace.moveWrist;
            }
                
                break;
            case moveWrist:
                if(!(armSubsystem.clawEncoder.getPosition() < 45)) {
                    armSubsystem.ClawRotation(0.3);
                }
                else {
                    timer.reset();
                    Constants.autoPlaceStage = Constants.eAutoPlace.ejectPiece;
                } 
                break;

            case ejectPiece:
                intakeSubsystem.intake.set(-0.4);
                Constants.autoPlaceStage = Constants.eAutoPlace.retractArm;
                break;
            case retractArm:
                armSubsystem.armSolenoid.set(Value.kForward);
                timer.reset();
                Constants.autoPlaceStage = Constants.eAutoPlace.stopOutake;
                break;
            
           
            
            case stopOutake:
                if(timer.hasElapsed(1)) { //2
                    intakeSubsystem.intake.set(0);
                    swerveDriveSubsystem.zeroWheelFunc();
                    Constants.autoPlaceStage = Constants.eAutoPlace.zeroWheel3;
                }
                break;
			case zeroWheel3:
                swerveDriveSubsystem.zeroWheelFunc();
                Constants.autoPlaceStage = Constants.eAutoPlace.driveAway;
                break;

            case driveAway:
            if(Math.abs(swerveDriveSubsystem.frontLeftModule.driveEncoder.getPosition()) <= 6.34) {
                swerveDriveSubsystem.driveRobotCentric(-0.2, 0, 0);
            }
            else {
                swerveDriveSubsystem.driveRobotCentric(0, 0, 0);
                Constants.autoPlaceStage = Constants.eAutoPlace.lowerArm;
            }
            break;


            case lowerArm: 
                if(armSubsystem.armEncoder.getPosition() < -135 ) {
                    armSubsystem.LiftArm(1);
                    if(armSubsystem.clawEncoder.getPosition() < 70) {
                        armSubsystem.ClawRotation(-0.3);
                    }
                    
                
                }
                else {
                    armSubsystem.LiftArm(0);
                    armSubsystem.ClawRotation(0);
                    Constants.autoPlaceStage = Constants.eAutoPlace.lowerArmToBalance;
                }
                break;

            case lowerArmToBalance:
                if(armSubsystem.armEncoder.getPosition() < -20) {
                    armSubsystem.LiftArm(1);
                    if(armSubsystem.clawEncoder.getPosition() < 70) {
                        armSubsystem.ClawRotation(-0.1);
                    }

                    swerveDriveSubsystem.driveRobotCentric(-0.45, 0, 0);
                }
                else {
                    armSubsystem.LiftArm(0);
                    armSubsystem.ClawRotation(0);
                    Constants.autoPlaceStage = Constants.eAutoPlace.driveToBalance;
                }
                break;
            case driveToBalance:
                SmartDashboard.putString("AUTOBALANCE", "FALSE");
                SmartDashboard.putNumber("PITCH", Constants.gyro.getPitch());
                if(OcrMath.isWithin(Constants.gyro.getPitch(), 10)) {
                    swerveDriveSubsystem.driveRobotCentric(-0.3, 0, 0);
                }
                else {
                    Constants.autoPlaceStage = Constants.eAutoPlace.gyroSmaller;
                }
                break;
            case gyroSmaller:
                if(OcrMath.isWithin(Constants.gyro.getPitch(), 24)) {
                    swerveDriveSubsystem.driveRobotCentric(-1, 0, 0);
                }
                else {
                    swerveDriveSubsystem.driveRobotCentric(0, 0, 0);
                    Constants.autoPlaceStage = Constants.eAutoPlace.autoBalance;
                }
                				
                break;
            case autoBalance:
                SmartDashboard.putString("AUTOBALANCE", "TRUE");
                autoBalance.execute();
                break;

                }

                
                //if(!timer.hasElapsed(1.5 + 0.3)) {
                //    swerveDriveSubsystem.driveRobotCentric(0, 0.5, 0);
               // }
                //else {
            //        Constants.autoPlaceStage = Constants.eAutoPlace.none;
             //       break;
                //}
             }
        //  }
    
    @Override
    public void execute() {
        SmartDashboard.putString("EvanCode", Constants.autoPlaceStage.toString());
        autoPlace();
    }
    @Override
    public void end(boolean interrupted) {
        armSubsystem.LiftArm(0);
        swerveDriveSubsystem.zeroWheelFunc();
        Constants.autoPlaceStage = Constants.eAutoPlace.zeroWheels;
        
        timer.reset();
        
    }
	@Override
	public boolean isFinished() {
		return Constants.autoPlaceStage == Constants.eAutoPlace.driveToBalance;
	}
    
}



/*   if(!timer.hasElapsed(0.3)) {
                    armSubsystem.armSolenoid.set(Value.kForward);
                    intakeSubsystem.intake.set(-.4);
                    SmartDashboard.putString("Hint", "wow");

                }
                else{
                    SmartDashboard.putString("Hint", "100");
                    timer.stop();
                    timer.reset();
                    intakeSubsystem.intake.set(0);
                    armSubsystem.ClawRotation(0);
                        SmartDashboard.putString("Hint", "done");
                        Constants.autoPlaceStage = Constants.eAutoPlace.none;
                    

                } */

                             /*  if(gyroPitch > Constants.gyro.getPitch()) {
                    swerveDriveSubsystem.driveRobotCentric(-0.3, 0, 0);
                }
                else {
                    swerveDriveSubsystem.driveRobotCentric(0, 0, 0);
                    Constants.autoPlaceStage = Constants.eAutoPlace.autoBalance;
                }
                gyroPitch = Constants.gyro.getPitch();*/