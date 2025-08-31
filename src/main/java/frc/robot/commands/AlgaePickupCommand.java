package frc.robot.commands;

import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;

public class AlgaePickupCommand extends SequentialCommandGroup {
    public AlgaePickupCommand(ArmSubsystem arm, ElevatorSubsystem elevator) {
        addCommands(
            elevator.moveToPosition(Constants.ElevatorConstants.ElevatorPosition.GROUNDALG),
            new WaitCommand(0.3),
            arm.moveToPosition(Constants.ArmConstants.ArmPositions.GROUND_ALGAE),
            arm.intakePiece().until(arm::hasPiece),
            new WaitCommand(0.25),
            arm.moveToPosition(Constants.ArmConstants.ArmPositions.DEFAULT)
        );
        
    }
}
