package frc.robot.commands;

import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;

public class PickupCoralCommand extends SequentialCommandGroup {
    public PickupCoralCommand(ArmSubsystem arm, ElevatorSubsystem elevator) {
        addCommands(
            arm.moveToPosition(Constants.ArmConstants.ArmPositions.PICKUP),
            elevator.moveToPosition(Constants.ElevatorConstants.ElevatorPosition.PICKUP),
            arm.intakePiece().until(arm::hasPiece) 
            

        );
        
    }
}
