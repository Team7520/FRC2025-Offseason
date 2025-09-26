package frc.robot.commands;

import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants.ArmPositions;

public class L4PlaceCommandAuto extends ParallelCommandGroup {
    public L4PlaceCommandAuto(ArmSubsystem arm, ElevatorSubsystem elevator) {
        addCommands(
            arm.moveWhileRolling(Constants.ArmConstants.ArmPositions.ALGAE)
        );  
    }
}
