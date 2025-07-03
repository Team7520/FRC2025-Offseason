package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.EndEffectorSubsystem;


public class L3Command extends SequentialCommandGroup {
    public L3Command(ElevatorSubsystem elevator, EndEffectorSubsystem endEffector, double conveyorSpeed) {
        addCommands(
            endEffector.setPivotPositionCommand(Constants.EndEffectorConstants.PivotPosition.DOWN),
            new WaitUntilCommand(() -> endEffector.handOut()),
            elevator.moveToPosition(Constants.ElevatorConstants.ElevatorPosition.MID),
            new WaitCommand(0.6)
            // endEffector.setConveyorSpeedCommand(conveyorSpeed)
            //     .withTimeout(2) // Run for 2 seconds
        );
    }
}