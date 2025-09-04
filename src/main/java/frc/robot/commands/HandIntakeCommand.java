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
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class HandIntakeCommand extends Command {

    ArmSubsystem arm;
    DoubleSupplier leftTrigger;
    Boolean cancelled = false;
    
    public HandIntakeCommand(ArmSubsystem arm, DoubleSupplier leftTrigger) {
        this.arm = arm;
        this.leftTrigger = leftTrigger;
    }

    @Override
    public void execute() {
        if(leftTrigger.getAsDouble() < 0.5) {
            cancelled = true;            
        } 
        
        if(!cancelled) {
            arm.intake();         
        }
    }

    @Override
    public void end(boolean interrupted) {
        if(arm.hasPiece()) {
            arm.intakePiece().schedule();
        } else {
            arm.stopOpenLoop();
        }
        // arm.captureHoldFromEncoder();
    }

    @Override
    public boolean isFinished() {
        return cancelled || arm.hasPiece();
    }
}

