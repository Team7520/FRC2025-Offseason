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
import frc.robot.Constants;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeCommand extends Command {

    IntakeSubsystem intake;
    DoubleSupplier leftTrigger;
    Boolean cancelled = false;
    Boolean inAuto = false;
    
    public IntakeCommand(IntakeSubsystem intake, DoubleSupplier leftTrigger, Boolean inAuto) {
        this.intake = intake;
        this.leftTrigger = leftTrigger;
        this.inAuto = inAuto;
    }

    @Override
    public void execute() {
        if(leftTrigger.getAsDouble() < 0.5 && !inAuto) {
            cancelled = true;            
        } 
        
        if(!cancelled) {
            intake.setPivotPositionCommand(Constants.IntakeConstants.PivotPosition.GROUND).schedule();
            new WaitCommand(2);
            intake.runIntake(1.5);
            
        }
    }

    @Override
    public void end(boolean interrupted) {
        if(!inAuto) {
            intake.setPivotPositionCommand(Constants.IntakeConstants.PivotPosition.UP).alongWith(new InstantCommand(() -> intake.stopAll())).schedule();
        }
        new InstantCommand(() -> intake.stopAll()).schedule();
        
    }

    @Override
    public boolean isFinished() {
        return cancelled || intake.inBasket();
    }
}

