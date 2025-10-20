package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.DeferredCommand;

import frc.robot.Constants.ApriltagConstants;
import frc.robot.commands.DriveToPoseCommand;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.AprilTagSystem;

import java.util.Set;

public class Align2RightCommand extends DeferredCommand {
  public Align2RightCommand(
      SwerveSubsystem drivebase,
      AprilTagSystem aprilTagSystem
  ) {
    super(() -> {
      Pose2d robotPose = drivebase.getPose();
      Pose2d tagPose = aprilTagSystem.getNearestTagPose(robotPose);
      if (tagPose == null) {
        System.out.println("No AprilTag found on field (right bumper)!");
        return new InstantCommand();
      }

      double xOffset = ApriltagConstants.xOffsetRight;
      double yOffset = ApriltagConstants.yOffsetRight;

      Pose2d offsetPose = aprilTagSystem.getOffsetPose(tagPose, xOffset, yOffset);
      Rotation2d facingTag = offsetPose.getRotation();
      Rotation2d facingAway = facingTag.rotateBy(Rotation2d.fromDegrees(180));

      Pose2d candidateFront = new Pose2d(offsetPose.getTranslation(), facingTag);
      Pose2d candidateBack  = new Pose2d(offsetPose.getTranslation(), facingAway);

      Pose2d optimalAlign = aprilTagSystem.getOptimalAlignPose(robotPose, candidateFront, candidateBack);

      System.out.println("Driving to OPTIMAL RIGHT align pose: " + optimalAlign);

      return new DriveToPoseCommand(drivebase, optimalAlign);
    }, Set.of(drivebase));
  }
}