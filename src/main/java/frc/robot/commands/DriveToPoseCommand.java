package frc.robot.commands;

import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;

public class DriveToPoseCommand extends Command {
    private final Command pathCommand;

    public DriveToPoseCommand(Pose2d currentPose, Pose2d targetPose) {

        List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(
            currentPose,
            targetPose
        );

        PathConstraints constraints = new PathConstraints(1, 1, 2 * Math.PI, 4 * Math.PI); // The constraints for this path.

        GoalEndState goalEndState = new GoalEndState(
            0.0, // end velocity (m/s)
            targetPose.getRotation()
        );

        PathPlannerPath path = new PathPlannerPath(
            waypoints,
            constraints,
            null,          // ideal start state (null for dynamic start)
            goalEndState   // your desired end rotation
        );
        path.preventFlipping = true;

        // Build the command to fo  llow this path
        pathCommand = AutoBuilder.followPath(path);

        addRequirements();
    }

    @Override
    public void initialize() {
        pathCommand.initialize();
    }

    @Override
    public void execute() {
        pathCommand.execute();
    }

    @Override
    public void end(boolean interrupted) {
        pathCommand.end(interrupted);
    }

    @Override
    public boolean isFinished() {
        return pathCommand.isFinished();
    }
}
