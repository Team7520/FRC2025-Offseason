package frc.robot.commands;

import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;

public class L1Command extends SequentialCommandGroup {
    public L1Command(ArmSubsystem arm, ElevatorSubsystem elevator, Boolean flip) {
        if(flip) {
            addCommands(
                elevator.moveToPosition(Constants.ElevatorConstants.ElevatorPosition.L1),   
                new WaitCommand(1),
                arm.moveToPosition(Constants.ArmConstants.ArmPositions.FLIPL1)
                //arm.eject();
            );
        } else {
            addCommands(
                elevator.moveToPosition(Constants.ElevatorConstants.ElevatorPosition.L1),   
                arm.moveToPosition(Constants.ArmConstants.ArmPositions.L1)
                //arm.eject();
            );
        }
        
        
    }
}
