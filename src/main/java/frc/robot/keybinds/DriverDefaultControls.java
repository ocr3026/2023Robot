package frc.robot.keybinds;

import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;

public class DriverDefaultControls {

  //change these to invert driving and turning
  public static boolean driveYIsInverted = false;
  public static boolean driveXIsInverted = false;
  public static boolean steerIsInverted = false;

  public void updateDriveValues() {

    public static double driveX = Constants.translationJoystick.getX();
    public static double driveY = Constants.translationJoystick.getY();
    public static double steer = Constants.rotationJoystick.getX();
    

    if(driveYIsInverted) {
      driveY = (Constants.translationJoystick.getY() * (-1));
    }
    if(driveXIsInverted) {
      driveX = (Constants.translationJoystick.getX() * (-1));
    }
    if(steerIsInverted) {
      steer = (Constants.rotationJoystick.getX() * -1)
    }
}

  public final Trigger fieldTrigger = Constants.translationJoystick.button(2);
  public final Trigger balanceTrigger = Constants.translationJoystick.button(3);
  public final Trigger zeroWheel = Constants.translationJoystick.button(10);
  public final Trigger slowerDrive = Constants.rotationJoystick.button(1);
  public final Trigger resetGyro = Constants.rotationJoystick.button(9);
  public final Trigger zeroClaw = Constants.rotationJoystick.button(8);
  public final Trigger zeroArm = Constants.rotationJoystick.button(7);

}

