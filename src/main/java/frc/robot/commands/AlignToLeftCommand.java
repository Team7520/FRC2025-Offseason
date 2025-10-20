package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ProxyCommand;

import frc.robot.Constants.ApriltagConstants;
import frc.robot.commands.DriveToPoseCommand;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.AprilTagSystem;

public class AlignToLeftCommand extends ProxyCommand {
  public AlignToLeftCommand(
      SwerveSubsystem drivebase,
      AprilTagSystem aprilTagSystem
  ) {
    super(() -> {
      Pose2d robotPose = drivebase.getPose();
      Pose2d tagPose = aprilTagSystem.getNearestTagPose(robotPose);
      if (tagPose == null) {
        System.out.println("No AprilTag found on field (left bumper)!");
        return new InstantCommand();
      }

      double xOffset = ApriltagConstants.xOffsetLeft;
      double yOffset = ApriltagConstants.yOffsetLeft;

      Pose2d offsetPose = aprilTagSystem.getOffsetPose(tagPose, xOffset, yOffset);
      Rotation2d facingTag = offsetPose.getRotation();
      Rotation2d facingAway = facingTag.rotateBy(Rotation2d.fromDegrees(180));

      Pose2d candidateFront = new Pose2d(offsetPose.getTranslation(), facingTag);
      Pose2d candidateBack  = new Pose2d(offsetPose.getTranslation(), facingAway);

      Pose2d optimalAlign = aprilTagSystem.getOptimalAlignPose(robotPose, candidateFront, candidateBack);

      System.out.println("Driving to OPTIMAL LEFT align pose: " + optimalAlign);
      System.out.println("Current POSE: " + robotPose);

      return new DriveToPoseCommand(drivebase, optimalAlign);
    });

    addRequirements(drivebase);
  }
}