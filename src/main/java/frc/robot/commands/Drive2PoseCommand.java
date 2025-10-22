package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.AprilTagSystem;
import frc.robot.Constants.ApriltagConstants;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class Drive2PoseCommand extends Command {

    SwerveSubsystem drivebase;
    AprilTagSystem aprilTagSystem;
    String side;
    int XOffset = 0;
    int YOffset = 0;
    
    public Drive2PoseCommand(SwerveSubsystem drivebase, AprilTagSystem aprilTagSystem, String side, int xOffset, int yOffset) {
        this.drivebase = drivebase;
        this.aprilTagSystem = aprilTagSystem;
        this.side = side;
        this.XOffset = XOffset;
        this.YOffset = YOffset;
    }

    @Override
    public void execute() {
        if(side.equals("right")) {
            Pose2d robotPose = drivebase.getPose();

            Pose2d tagPose = aprilTagSystem.getNearestTagPose(robotPose);
        
            double xOffset = ApriltagConstants.xOffsetRight + XOffset;
            double yOffset = ApriltagConstants.yOffsetRight + YOffset;
    
            // Find translation for right side
            Pose2d offsetPose = aprilTagSystem.getOffsetPose(tagPose, xOffset, yOffset);
            Rotation2d facingTag = offsetPose.getRotation();
            Rotation2d facingAway = facingTag.rotateBy(Rotation2d.fromDegrees(180));
    
            Pose2d candidateFront = new Pose2d(offsetPose.getTranslation(), facingTag);
            Pose2d candidateBack  = new Pose2d(offsetPose.getTranslation(), facingAway);
    
            Pose2d optimalAlign = aprilTagSystem.getOptimalAlignPose(robotPose, candidateFront, candidateBack);
        
            new DriveToPoseCommand(
                drivebase,
                optimalAlign
            );
        } else if(side.equals("left")) {
            Pose2d robotPose = drivebase.getPose();
            Pose2d tagPose = aprilTagSystem.getNearestTagPose(robotPose);
    
            double xOffset = ApriltagConstants.xOffsetLeft + XOffset;
            double yOffset = ApriltagConstants.yOffsetLeft + YOffset;
    
            // Find translation for left side
            Pose2d offsetPose = aprilTagSystem.getOffsetPose(tagPose, xOffset, yOffset);
            Rotation2d facingTag = offsetPose.getRotation();
            Rotation2d facingAway = facingTag.rotateBy(Rotation2d.fromDegrees(180));
    
            Pose2d candidateFront = new Pose2d(offsetPose.getTranslation(), facingTag);
            Pose2d candidateBack  = new Pose2d(offsetPose.getTranslation(), facingAway);
    
            Pose2d optimalAlign = aprilTagSystem.getOptimalAlignPose(robotPose, candidateFront, candidateBack);
    
            new DriveToPoseCommand(
                drivebase, 
                optimalAlign
            );
        }   
    }
}