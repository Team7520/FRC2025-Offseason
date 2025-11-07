package frc.robot.commands;

import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;

public class L4Command extends SequentialCommandGroup {
    public L4Command(ArmSubsystem arm, ElevatorSubsystem elevator, Boolean flip, Boolean inAuto) {
            if(flip) {
                addCommands(
                    elevator.moveToPosition(Constants.ElevatorConstants.ElevatorPosition.L4) ,  
                    new WaitCommand(1), 
                    arm.moveToPosition(Constants.ArmConstants.ArmPositions.L4)               
                    // arm.eject()
                );
            } else {
                if(inAuto) {
                    addCommands(
                        new ParallelCommandGroup(
                            arm.moveToPosition(Constants.ArmConstants.ArmPositions.L4),
                            new SequentialCommandGroup(
                                new WaitCommand(0.1),
                                elevator.moveToPosition(Constants.ElevatorConstants.ElevatorPosition.L4First),
                                new WaitCommand(0.8),
                                elevator.moveToPosition(Constants.ElevatorConstants.ElevatorPosition.L4)
                            )
                        )
                    );
                } else {
                    addCommands(
                        new ParallelCommandGroup(
                            arm.moveToPosition(Constants.ArmConstants.ArmPositions.PREL4),
                            new SequentialCommandGroup(
                                new WaitCommand(0.1),
                                elevator.moveToPosition(Constants.ElevatorConstants.ElevatorPosition.L4First),
                                new WaitCommand(0.7),
                                elevator.moveToPosition(Constants.ElevatorConstants.ElevatorPosition.L4)
                            )
                        ),
                        new WaitCommand(0.5406),
                        arm.moveToPosition(Constants.ArmConstants.ArmPositions.L4)          
                        // arm.eject()
                    );
                }
            }    
        
    }
}

