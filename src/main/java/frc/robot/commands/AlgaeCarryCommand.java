package frc.robot.commands;

import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;

public class AlgaeCarryCommand extends ParallelCommandGroup {
    public AlgaeCarryCommand(ArmSubsystem arm, ElevatorSubsystem elevator) { 
            addCommands(
                elevator.moveToPosition(Constants.ElevatorConstants.ElevatorPosition.GROUND),
                arm.moveToPositionWithSpeed(Constants.ArmConstants.ArmPositions.DEFAULT,0.5)
            );
    }
}

