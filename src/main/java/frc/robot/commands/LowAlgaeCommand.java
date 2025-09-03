package frc.robot.commands;

import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorConstants;

public class LowAlgaeCommand extends SequentialCommandGroup {
    public LowAlgaeCommand(ArmSubsystem arm, ElevatorSubsystem elevator) {
        if(elevator.getPositionDouble() <= Constants.ElevatorConstants.ElevatorPosition.LOWALG.getHeight()) {
            addCommands(
                elevator.moveToPosition(Constants.ElevatorConstants.ElevatorPosition.LOWALG),
                arm.moveToPosition(Constants.ArmConstants.ArmPositions.ALGAE)
                
            );
        } else {
            addCommands(  
                arm.moveToPosition(Constants.ArmConstants.ArmPositions.ALGAE),
                elevator.moveToPosition(Constants.ElevatorConstants.ElevatorPosition.LOWALG)
                // new WaitCommand(0.7),
                // arm.moveToPosition(Constants.ArmConstants.ArmPositions.DEFAULT)
        );
        }
        
    }
}
