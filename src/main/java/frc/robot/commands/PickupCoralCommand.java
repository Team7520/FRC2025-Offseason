package frc.robot.commands;

import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;

public class PickupCoralCommand extends SequentialCommandGroup {
    public PickupCoralCommand(ArmSubsystem arm, ElevatorSubsystem elevator, Boolean inAuto) {
        if(inAuto) {
            addCommands(
                new ParallelCommandGroup(
                    arm.intakePiece(),
                    elevator.moveToPosition(Constants.ElevatorConstants.ElevatorPosition.PICKUP)
                ),
                elevator.moveToPosition(Constants.ElevatorConstants.ElevatorPosition.READY)
            );    
        } else {
            if(arm.atTarget(Constants.ArmConstants.ArmPositions.PICKUP)) {
                System.out.print("At target!");
                addCommands(
                    new ParallelCommandGroup(
                        arm.intakePiece(),
                        elevator.moveToPosition(Constants.ElevatorConstants.ElevatorPosition.PICKUP)
                    ),  
                    elevator.moveToPosition(Constants.ElevatorConstants.ElevatorPosition.READY)
                    // arm.moveToPosition(Constants.ArmConstants.ArmPositions.DEFAULT),
                    // elevator.moveToPosition(Constants.ElevatorConstants.ElevatorPosition.L2SCORE)
                );    
            } else {
                addCommands(
                    new ReadyToPickupCommand(arm, elevator),
                    new WaitCommand(0.3),
                    new ParallelCommandGroup(
                        elevator.moveToPosition(Constants.ElevatorConstants.ElevatorPosition.PICKUP),
                        arm.intakePiece()
                    ),
                    elevator.moveToPosition(Constants.ElevatorConstants.ElevatorPosition.READY)
                    // arm.moveToPosition(Constants.ArmConstants.ArmPositions.DEFAULT),
                    // elevator.moveToPosition(Constants.ElevatorConstants.ElevatorPosition.L2SCORE)
                );  
            }
        }
    }
}
