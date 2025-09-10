package frc.robot.commands;

import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants.ArmPositions;

public class L2PlaceCommand extends SequentialCommandGroup {
    public L2PlaceCommand(ArmSubsystem arm, ElevatorSubsystem elevator, Boolean flip) {
        if(flip) {
            addCommands(
                arm.moveToPosition(Constants.ArmConstants.ArmPositions.FLIPALGAE),
                arm.ejectPiece(1),
                new WaitCommand(0.5),
                elevator.moveToPosition(Constants.ElevatorConstants.ElevatorPosition.L2SCORE)
            );
        } else {
            addCommands(
                arm.moveToPosition(Constants.ArmConstants.ArmPositions.ALGAE),
                arm.ejectPiece(1),
                new WaitCommand(0.5),
                elevator.moveToPosition(Constants.ElevatorConstants.ElevatorPosition.L2SCORE)
            );
        }
        
    }
}
