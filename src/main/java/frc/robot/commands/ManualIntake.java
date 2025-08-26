package frc.robot.commands;

import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj2.command.Command;
//import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class ManualIntake extends Command {
    private final IntakeSubsystem intakeSubsystem;
    private final DoubleSupplier input;
    private static final double MANUAL_ADJUSTMENT_RATE = 2; // Rotations per execution

    public ManualIntake(IntakeSubsystem intakeSubsystem, DoubleSupplier input) {
        this.intakeSubsystem = intakeSubsystem;
        this.input = input;
        addRequirements(intakeSubsystem);
    }

    @Override
    public void execute() {
        double adjustedInput = input.getAsDouble() * MANUAL_ADJUSTMENT_RATE;
        if (Math.abs(adjustedInput) > 0.2) { // Add deadband
            intakeSubsystem.manual(adjustedInput);
        } else {
            
        }
    }
}
