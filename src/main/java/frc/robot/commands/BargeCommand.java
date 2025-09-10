package frc.robot.commands;

import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorConstants;

public class BargeCommand extends SequentialCommandGroup {
    public BargeCommand(ArmSubsystem arm, ElevatorSubsystem elevator, Boolean flip) {
        if(flip) {
            addCommands(
                arm.moveToPosition(Constants.ArmConstants.ArmPositions.DEFAULT),
                elevator.moveToPosition(Constants.ElevatorConstants.ElevatorPosition.BARGE),
                new WaitCommand(0.3),
                arm.moveToPosition(Constants.ArmConstants.ArmPositions.FLIPBARGE)        
            );
        } else {
            addCommands(
                arm.moveToPosition(Constants.ArmConstants.ArmPositions.DEFAULT),
                elevator.moveToPosition(Constants.ElevatorConstants.ElevatorPosition.BARGE),
                new WaitCommand(0.3),
                arm.moveToPosition(Constants.ArmConstants.ArmPositions.BARGE)        
            );
        }
        
    }
}
