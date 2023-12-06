package frc.robot.keybinds;

import edu.wpi.first.wpilibj.PS4Controller.Button;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;

public final class ManipulatorDefaultControls {
    public static final Trigger extendArm = Constants.xboxController.b();
    public static final Trigger retractArm = Constants.xboxController.x();
    public static final Trigger clawIntakTrigger = Constants.xboxController.leftTrigger();
	public static final Trigger clawOutakeTrigger = Constants.xboxController.rightTrigger();
    public static final Trigger Brake =	Constants.xboxController.button(7);
    public static final Trigger Coast = Constants.xboxController.button(8);



    public static boolean invertArm = false;
    public static boolean invertClaw = false;

    public void updateControllers() {

        public static double liftArm = Constants.xboxController.getLeftY();
        public static double rotateClaw = Constants.xboxController.getRightY();
        
        if(invertArm) {
            liftArm = (Constants.xboxController.getLeftY() * -1) 
        }

        if(invertClaw) {
        rotateClaw = (Constants.xboxController.getRightY() * -1)
        }
    }

    




}
