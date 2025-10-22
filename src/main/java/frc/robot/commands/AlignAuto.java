package frc.robot.commands;

import java.util.Arrays;
import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.ConstraintsZone;
import com.pathplanner.lib.path.EventMarker;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PointTowardsZone;
import com.pathplanner.lib.path.RotationTarget;
import com.pathplanner.lib.path.Waypoint;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class AlignAuto extends Command {
    private final Pose2d targetPose;
    private final SwerveSubsystem drivebase;
    private Command sequentialCommand;

    public AlignAuto(
        SwerveSubsystem drivebase,
        Pose2d targetPose
    ) {
        this.drivebase = drivebase;
        this.targetPose = targetPose;
        
        addRequirements(drivebase);
    }

    @Override
    public void initialize() {
        Pose2d currentPose = drivebase.getPose();
        
        List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(
            currentPose,
            targetPose
        );

        PathConstraints constraints = new PathConstraints(2, 2, 4 * Math.PI, 4 * Math.PI);

        GoalEndState goalEndState = new GoalEndState(
            0.0,
            targetPose.getRotation()
        );

        ConstraintsZone zones = new ConstraintsZone(50, 100, new PathConstraints(1, 1, 4 * Math.PI, 4 * Math.PI));
        List<RotationTarget> lst_rt = Arrays.asList();
        List<ConstraintsZone> lst_cz = Arrays.asList(zones);
        List<PointTowardsZone> lst_ptz = Arrays.asList();
        List<EventMarker> lst_em = Arrays.asList();

        PathPlannerPath path = new PathPlannerPath(
            waypoints,
            lst_rt,
            lst_ptz,
            lst_cz,
            lst_em,
            constraints,
            null,
            goalEndState,
            false
        );
        path.preventFlipping = true;

        Command pathCommand = AutoBuilder.followPath(path);
        Command turnCommand = new TurnToAngleCommand(drivebase, targetPose.getRotation());
        
        // Run PathPlanner first (which handles both position and rotation)
        // Then run turn command to clean up any rotation error
        sequentialCommand = new SequentialCommandGroup(
            pathCommand,
            turnCommand
        );
        
        sequentialCommand.initialize();
    }

    @Override
    public void execute() {
        sequentialCommand.execute();
    }

    @Override
    public void end(boolean interrupted) {
        sequentialCommand.end(interrupted);
    }

    @Override
    public boolean isFinished() {
        return sequentialCommand.isFinished();
    }

    public static class TurnToAngleCommand extends Command {
        private final SwerveSubsystem drivebase;
        private final Rotation2d targetAngle;
        private final PIDController rotationController;
        
        public TurnToAngleCommand(SwerveSubsystem drivebase, Rotation2d targetAngle) {
            this.drivebase = drivebase;
            this.targetAngle = targetAngle;
            this.rotationController = new PIDController(4.0, 0.0, 0.1); // Tune these values
            rotationController.enableContinuousInput(-Math.PI, Math.PI);
            rotationController.setTolerance(Math.toRadians(1));
            
            addRequirements(drivebase);
        }
        
        @Override
        public void initialize() {
            System.out.println("Starting precise rotation to: " + targetAngle.getDegrees() + " degrees");
        }
        
        @Override
        public void execute() {
            double rotationSpeed = rotationController.calculate(
                drivebase.getPose().getRotation().getRadians(),
                targetAngle.getRadians()
            );
            
            drivebase.drive(new Translation2d(0,0), rotationSpeed, true);
        }
        
        @Override
        public boolean isFinished() {
            return rotationController.atSetpoint();
        }
        
        @Override
        public void end(boolean interrupted) {
            drivebase.drive(new Translation2d(0,0), 0, true);
            
        }
    }
}