package frc.robot.commands;

import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;

public class HighAlgaeCommand extends SequentialCommandGroup {
    public HighAlgaeCommand(ArmSubsystem arm, ElevatorSubsystem elevator, Boolean flip) {
        if(flip) {
            addCommands(
                elevator.moveToPosition(Constants.ElevatorConstants.ElevatorPosition.HIGHALG),
                new WaitCommand(0.3),
                arm.moveToPosition(Constants.ArmConstants.ArmPositions.FLIPALGAE)
                // new WaitCommand(0.7),
                // arm.moveToPosition(Constants.ArmConstants.ArmPositions.DEFAULT)
            );
        } else {
            addCommands(
                elevator.moveToPosition(Constants.ElevatorConstants.ElevatorPosition.HIGHALG),
                new WaitCommand(0.3),
                arm.moveToPosition(Constants.ArmConstants.ArmPositions.ALGAE)
                // new WaitCommand(0.7),
                // arm.moveToPosition(Constants.ArmConstants.ArmPositions.DEFAULT)
            );
        }
        
        
    }
}
