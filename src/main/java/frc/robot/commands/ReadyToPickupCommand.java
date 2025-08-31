package frc.robot.commands;

import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;

public class ReadyToPickupCommand extends SequentialCommandGroup {
    public ReadyToPickupCommand(ArmSubsystem arm, ElevatorSubsystem elevator) {
        addCommands(
            elevator.moveToPosition(Constants.ElevatorConstants.ElevatorPosition.READY),  
            new WaitCommand(0.6),
            arm.moveToPosition(Constants.ArmConstants.ArmPositions.PICKUP)

        );
        
    }
}
