package frc.robot.commands;

import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;

public class IntakeCommand extends SequentialCommandGroup {
    public IntakeCommand(IntakeSubsystem intake) {
        addCommands(
            intake.setPivotPositionCommand(Constants.IntakeConstants.PivotPosition.GROUND),
            intake.intakePiece(),
            intake.setPivotPositionCommand(Constants.IntakeConstants.PivotPosition.UP)
        );  
    }
}
