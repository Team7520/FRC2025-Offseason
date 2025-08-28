package frc.robot.commands;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ArmSubsystem;

public class IntakeCommand extends Command {
    private final IntakeSubsystem intake;
    private final ArmSubsystem arm;
    private final Supplier<String> modeSupplier;

    public IntakeCommand(IntakeSubsystem intake, ArmSubsystem arm, Supplier<String> modeSupplier) {
        this.intake = intake;
        this.arm = arm;
        this.modeSupplier = modeSupplier;
        addRequirements(intake, arm);
    }

    @Override
    public void execute() {
        if (modeSupplier.get().equals("Coral")) {
            intake.runIntake(1);
        } else {
            arm.intake();
        }
    }

    @Override
    public boolean isFinished() {
        return !modeSupplier.get().equals("Coral") && arm.hasPiece();
    }

    @Override
    public void end(boolean interrupted) {
        if (!modeSupplier.get().equals("Coral")) {
            arm.captureHoldFromEncoder();
        }
        intake.stopAll();
    }
}
