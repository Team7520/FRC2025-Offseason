package frc.robot.commands;

import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;

public class PickupCoralCommand extends SequentialCommandGroup {
    public PickupCoralCommand(ArmSubsystem arm, ElevatorSubsystem elevator, Boolean inAuto) {
        if(inAuto) {
            addCommands(
                arm.moveToPosition(Constants.ArmConstants.ArmPositions.PICKUP),
                elevator.moveToPosition(Constants.ElevatorConstants.ElevatorPosition.PICKUP),
                arm.intakePiece(),
                elevator.moveToPosition(Constants.ElevatorConstants.ElevatorPosition.READY),
                new WaitCommand(0.4)
            );    
        } else {
            if(arm.atTarget(Constants.ArmConstants.ArmPositions.PICKUP)) {
                addCommands(
                    elevator.moveToPosition(Constants.ElevatorConstants.ElevatorPosition.PICKUP),
                    arm.intakePiece(),
                    elevator.moveToPosition(Constants.ElevatorConstants.ElevatorPosition.READY),
                    new WaitCommand(0.7520),
                    arm.moveToPosition(Constants.ArmConstants.ArmPositions.DEFAULT),
                    elevator.moveToPosition(Constants.ElevatorConstants.ElevatorPosition.L2SCORE)
                );    
            } else {
                addCommands(
                    arm.moveToPosition(Constants.ArmConstants.ArmPositions.PICKUP),
                    elevator.moveToPosition(Constants.ElevatorConstants.ElevatorPosition.PICKUP),
                    arm.intakePiece(),
                    elevator.moveToPosition(Constants.ElevatorConstants.ElevatorPosition.READY),
                    new WaitCommand(0.7520),
                    arm.moveToPosition(Constants.ArmConstants.ArmPositions.DEFAULT),
                    elevator.moveToPosition(Constants.ElevatorConstants.ElevatorPosition.L2SCORE)
                );  
            }
        }
    }
}
