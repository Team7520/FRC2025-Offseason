package frc.robot.commands;

import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorConstants;

public class LowAlgaeCommand extends SequentialCommandGroup {
    public LowAlgaeCommand(ArmSubsystem arm, ElevatorSubsystem elevator, Boolean flip) {
        if(elevator.getPositionDouble() <= Constants.ElevatorConstants.ElevatorPosition.LOWALG.getHeight()) {
            if(flip) {
                addCommands(
                    elevator.moveToPosition(Constants.ElevatorConstants.ElevatorPosition.LOWALG),
                    new WaitCommand(0.7),
                    arm.moveToPosition(Constants.ArmConstants.ArmPositions.FLIPALGAE)
                );
            } else {
                addCommands(
                    elevator.moveToPosition(Constants.ElevatorConstants.ElevatorPosition.LOWALG),
                    new WaitCommand(0.7),
                    arm.moveToPosition(Constants.ArmConstants.ArmPositions.ALGAE)
                );
            }
            
        } else {
            if(flip) {
                addCommands(  
                    arm.moveToPosition(Constants.ArmConstants.ArmPositions.FLIPALGAE),
                    elevator.moveToPosition(Constants.ElevatorConstants.ElevatorPosition.LOWALG)
                );
            } else {
                addCommands(
                    arm.moveToPosition(Constants.ArmConstants.ArmPositions.ALGAE),
                    elevator.moveToPosition(Constants.ElevatorConstants.ElevatorPosition.LOWALG)
                );
            }
            
        }
        
    }
}
