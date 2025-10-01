package frc.robot.commands;

import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;

public class LowAlgaeAuto extends SequentialCommandGroup {
    public LowAlgaeAuto(ArmSubsystem arm, ElevatorSubsystem elevator) {
                addCommands(
                    elevator.moveToPosition(Constants.ElevatorConstants.ElevatorPosition.LOWALG) ,  
                    arm.moveToPosition(Constants.ArmConstants.ArmPositions.ALGAE)               
                        // arm.eject()
                );
            };    
}


