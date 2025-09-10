package frc.robot.commands;

import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;

public class L3Command extends SequentialCommandGroup {
    public L3Command(ArmSubsystem arm, ElevatorSubsystem elevator, Boolean flip) {
        if ((elevator.getPositionDouble() < Constants.ElevatorConstants.ElevatorPosition.READY.getHeight()) && (arm.getPositionDouble() > Constants.ArmConstants.ArmPositions.MOVEPOSSIBLE.getPosition())) {
            if(flip) {
                addCommands(
                    elevator.moveAndWaitToPosition(Constants.ElevatorConstants.ElevatorPosition.READY),
                    arm.moveToPosition(Constants.ArmConstants.ArmPositions.FLIPL2_3),
                    elevator.moveToPosition(Constants.ElevatorConstants.ElevatorPosition.L3)
                    // arm.eject()
                );
            } else {
                addCommands(
                    elevator.moveAndWaitToPosition(Constants.ElevatorConstants.ElevatorPosition.READY),
                    arm.moveToPosition(Constants.ArmConstants.ArmPositions.L2_3),
                    elevator.moveToPosition(Constants.ElevatorConstants.ElevatorPosition.L3)
                    // arm.eject()
                );
            }    
        }else{
            if(flip) {
                addCommands(
                    arm.moveToPosition(Constants.ArmConstants.ArmPositions.FLIPL2_3),
                    elevator.moveToPosition(Constants.ElevatorConstants.ElevatorPosition.L3)
                    // arm.eject()
                );
            } else {
                addCommands(
                    arm.moveToPosition(Constants.ArmConstants.ArmPositions.L2_3),
                    elevator.moveToPosition(Constants.ElevatorConstants.ElevatorPosition.L3)
                    // arm.eject()
                );
            }    
        }
    }
}

