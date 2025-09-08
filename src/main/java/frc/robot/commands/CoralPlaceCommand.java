package frc.robot.commands;

import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;

public class CoralPlaceCommand extends ParallelCommandGroup {
    public CoralPlaceCommand(ArmSubsystem arm, ElevatorSubsystem elevator) {
        addCommands(
            elevator.moveToPosition(Constants.ElevatorConstants.ElevatorPosition.L3SCORE),//elevator move down from that level
            arm.ejectPiece(0.2),
            arm.moveToPosition(Constants.ArmConstants.ArmPositions.ALGAE)
        );
        
    }
}
