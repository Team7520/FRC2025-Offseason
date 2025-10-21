package frc.robot.commands;

import java.util.List;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.AprilTagSystem;
import frc.robot.Constants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class Drive2PoseCommand extends Command {

    SwerveSubsystem drivebase;
    AprilTagSystem aprilTagSystem;
    
    public Drive2PoseCommand(SwerveSubsystem drivebase, AprilTagSystem aprilTagSystem) {
        this.drivebase = drivebase;
        this.aprilTagSystem = aprilTagSystem;
        
    }

    @Override
    public void execute() {
        // Pose2d tagPose = aprilTagSystem.getNearestTagPose(robotPose);
    
        //     double xOffset = ApriltagConstants.xOffsetRight;
        //     double yOffset = ApriltagConstants.yOffsetRight;
    
        //     // Find translation for right side
        //     Pose2d offsetPose = aprilTagSystem.getOffsetPose(tagPose, xOffset, yOffset);
        //     Rotation2d facingTag = offsetPose.getRotation();
        //     Rotation2d facingAway = facingTag.rotateBy(Rotation2d.fromDegrees(180));
    
        //     Pose2d candidateFront = new Pose2d(offsetPose.getTranslation(), facingTag);
        //     Pose2d candidateBack  = new Pose2d(offsetPose.getTranslation(), facingAway);
    
        //     Pose2d optimalAlign = aprilTagSystem.getOptimalAlignPose(robotPose, candidateFront, candidateBack);
    
        //     System.out.println("Driving to OPTIMAL RIGHT align pose: " + optimalAlign);
    
        //     return new DriveToPoseCommand(
        //         drivebase,
        //         optimalAlign
        //     );
    }
}