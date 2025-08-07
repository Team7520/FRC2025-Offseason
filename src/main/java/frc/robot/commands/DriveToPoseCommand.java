package frc.robot.commands;

import java.util.List;
import java.util.function.DoubleSupplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;

public class DriveToPoseCommand extends Command {
    private final Command pathCommand;
    private final DoubleSupplier xInput;
    private final DoubleSupplier yInput;
    private final double overrideThreshold = 0.2;
    private boolean wasCanceled = false;

    public DriveToPoseCommand(
        Pose2d currentPose,
        Pose2d targetPose,
        DoubleSupplier xInput,
        DoubleSupplier yInput
    ) {
        this.xInput = xInput;
        this.yInput = yInput;

        List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(
            currentPose,
            targetPose
        );

        PathConstraints constraints = new PathConstraints(2, 2, 2 * Math.PI, 4 * Math.PI);

        GoalEndState goalEndState = new GoalEndState(
            0.0,
            targetPose.getRotation()
        );

        PathPlannerPath path = new PathPlannerPath(
            waypoints,
            constraints,
            null,
            goalEndState
        );
        path.preventFlipping = true;

        pathCommand = AutoBuilder.followPath(path);
    }

    @Override
    public void initialize() {
        pathCommand.initialize();
    }

    @Override
    public void execute() {
        if (Math.abs(xInput.getAsDouble()) > overrideThreshold || Math.abs(yInput.getAsDouble()) > overrideThreshold) {
            System.out.println("DriveToPoseCommand: Joystick override detected. Cancelling.");
            wasCanceled = true;
        }

        if (!wasCanceled) {
            pathCommand.execute();
        }
    }

    @Override
    public void end(boolean interrupted) {
        pathCommand.end(interrupted || wasCanceled);
    }

    @Override
    public boolean isFinished() {
        return wasCanceled || pathCommand.isFinished();
    }
}
