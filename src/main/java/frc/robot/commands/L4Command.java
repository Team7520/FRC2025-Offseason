package frc.robot.commands;

import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;

public class L4Command extends SequentialCommandGroup {
    public L4Command(ArmSubsystem arm, ElevatorSubsystem elevator) {
        if (elevator.getPositionDouble() < Constants.ElevatorConstants.ElevatorPosition.READY.getHeight()) {
            addCommands(
            elevator.moveAndWaitToPosition(Constants.ElevatorConstants.ElevatorPosition.READY),
            arm.moveToPosition(Constants.ArmConstants.ArmPositions.L4),
            elevator.moveToPosition(Constants.ElevatorConstants.ElevatorPosition.L4)
            // arm.eject()
        );
        }else{
            addCommands(
            arm.moveToPosition(Constants.ArmConstants.ArmPositions.L4),
            elevator.moveToPosition(Constants.ElevatorConstants.ElevatorPosition.L4)
            // arm.eject()
        );}
    }
}

