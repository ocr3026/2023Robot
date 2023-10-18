package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.SwerveDriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;


public class AutoPlace extends CommandBase {
	Timer timer = new Timer();
	SwerveDriveSubsystem swerveDriveSubsystem;
	ArmSubsystem armSubsystem;
	IntakeSubsystem intakeSubsystem;
    VisionSubsystem visionSubsystem;

	PIDController autoBalancePID = new PIDController(0.0075, 0, 0);

    public AutoPlace(SwerveDriveSubsystem subsystem, ArmSubsystem arm, IntakeSubsystem intake, VisionSubsystem vision) {
        addRequirements(subsystem);
        swerveDriveSubsystem = subsystem;
        addRequirements(arm);
        armSubsystem = arm;
        addRequirements(intake);
        intakeSubsystem = intake;
        addRequirements(vision);
        visionSubsystem = vision;
        SmartDashboard.putString("Hint", "Bad");
    }

    @Override 
    public void initialize() {
        Constants.autoPlaceStage = Constants.eAutoPlace.raiseArm;
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

            case raiseArm:
                if(armSubsystem.armEncoder.getPosition() > -266) 
                {
                    armSubsystem.LiftArm(-1);
                    //System.out.println(armSubsystem.armMotor.getOutputCurrent());
                    if(!(armSubsystem.armMotor.getOutputCurrent() > 18)) {
                    if(!(armSubsystem.armEncoder.getPosition() > -90)) {
                        armSubsystem.ClawRotation(-0.2);
                    }
                    else if(armSubsystem.armEncoder.getPosition() > -90) {
                        armSubsystem.ClawRotation(0.1);
                    }
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
                    Constants.autoPlaceStage = Constants.eAutoPlace.driveForward;
                }
                
                break;

            case driveForward:
            if (swerveDriveSubsystem.frontLeftModule.driveEncoder.getPosition() > -8.84) {
                swerveDriveSubsystem.driveRobotCentric(.2 , 0 , 0);
            }
            else {
                swerveDriveSubsystem.driveRobotCentric(0, 0, 0);
                //Constants.autoPlaceStage = Constants.eAutoPlace.placePiece;
            }
                
                break;
          /*   case placePiece:
                if(!(armSubsystem.clawEncoder.getPosition() < 45)) {
                    armSubsystem.ClawRotation(0.1);
                    timer.reset();
                }
                else if(!timer.hasElapsed(0.5)) {
                    armSubsystem.ReturnArm();
                    intakeSubsystem.intake.set(-.4);
                    SmartDashboard.putString("Hint", "wow");

                }
                else{
                    timer.stop();
                    timer.reset();
                    intakeSubsystem.intake.set(0);
                    armSubsystem.ClawRotation(0);
                    if(timer.hasElapsed(1)) {
                        SmartDashboard.putString("Hint", "done");
                        Constants.autoPlaceStage = Constants.eAutoPlace.none;
                    }

                }
                break;
*/
            case driveToBalance:
                timer.reset();
                timer.start();
                //if(!timer.hasElapsed(1.5)) {
                    visionSubsystem.moveToPose2DCommand(null);
                    if(armSubsystem.armEncoder.getPosition() < -6) {
                        armSubsystem.LiftArm(0.3);
                        if(armSubsystem.clawEncoder.getPosition() < 70) {
                        armSubsystem.ClawRotation(-0.4);
                        }
                    }
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
        System.out.println(Constants.autoPlaceStage);
        SmartDashboard.putString("EvanCode", Constants.autoPlaceStage.toString());
        autoPlace();
    }
    @Override
    public void end(boolean interrupted) {
        armSubsystem.LiftArm(0);
    }
    
}

