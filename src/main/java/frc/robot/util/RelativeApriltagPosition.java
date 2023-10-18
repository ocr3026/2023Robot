package frc.robot.util;

import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveDriveSubsystem;

public class RelativeApriltagPosition {
    SwerveDriveSubsystem swerveDriveSubsystem;
    PIDController xPidController = new PIDController(0.01, 0, 0);
    PIDController zRotateController = new PIDController(0.01, 0, 0);

    public void relativeApirltagPosition(double xDist, double zRotation) {
        PhotonPipelineResult result = Constants.camera.getLatestResult();
        if(result.hasTargets()) {
            PhotonTrackedTarget target =  result.getBestTarget();
            if(target.getBestCameraToTarget().getX() != xDist) {
                swerveDriveSubsystem.driveRobotCentric(OcrMath.clamp(xPidController.calculate(target.getBestCameraToTarget().getX(), xDist), -0.3, 0.3), 0, 0);
                if(Units.radiansToDegrees(target.getBestCameraToTarget().getRotation().getZ()) != zRotation) {
                    swerveDriveSubsystem.driveRobotCentric(0, 0, OcrMath.clamp(zRotateController.calculate(Units.radiansToDegrees(target.getBestCameraToTarget().getRotation().getZ())), -0.2, 0.2));
                }
            }

        }
    }
}   
