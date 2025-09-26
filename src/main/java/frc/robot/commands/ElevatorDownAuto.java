package frc.robot.commands;

import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;

public class ElevatorDownAuto extends SequentialCommandGroup {
    public ElevatorDownAuto(ArmSubsystem arm, ElevatorSubsystem elevator) {
        addCommands(
            elevator.moveToPosition(Constants.ElevatorConstants.ElevatorPosition.READY)
        );
    }
}
