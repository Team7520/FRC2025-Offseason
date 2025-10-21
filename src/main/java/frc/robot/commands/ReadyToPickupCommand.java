package frc.robot.commands;

import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;

public class ReadyToPickupCommand extends SequentialCommandGroup {
    public ReadyToPickupCommand(ArmSubsystem arm, ElevatorSubsystem elevator) {
        if(elevator.getPositionDouble() > Constants.ElevatorConstants.ElevatorPosition.READY.getHeight()) {
            addCommands(
                elevator.moveToPosition(Constants.ElevatorConstants.ElevatorPosition.READY),  
                new WaitCommand(1.2),
                arm.moveToPosition(Constants.ArmConstants.ArmPositions.PICKUP)

            );
        } else {
            addCommands(
                elevator.moveToPosition(Constants.ElevatorConstants.ElevatorPosition.READY),  
                new WaitCommand(0.7),
                arm.moveToPosition(Constants.ArmConstants.ArmPositions.PICKUP)

            );
        }
        
    }
}
