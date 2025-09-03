package frc.robot.commands;

import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants.ArmPositions;

public class L4PlaceCommand extends SequentialCommandGroup {
    public L4PlaceCommand(ArmSubsystem arm, ElevatorSubsystem elevator) {
        addCommands(
            arm.moveToPosition(Constants.ArmConstants.ArmPositions.ALGAE),
            arm.ejectPiece(0.5)
        );
    }
}
