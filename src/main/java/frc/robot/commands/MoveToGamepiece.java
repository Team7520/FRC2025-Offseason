package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.CoralDetectionSystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import edu.wpi.first.math.geometry.Translation2d;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Rotation2d;

/**
 * Assisted drive: driver controls the robot, but vision steers left/right to line up with the gamepiece.
 * Exits when intake detects a gamepiece in the basket.
 */
public class MoveToGamepiece extends Command {

    private final SwerveSubsystem swerve;
    private final CoralDetectionSystem coralDetection;
    private final IntakeSubsystem intake;
    private static final double turningConstant = 0.09;
    private static final double coralCenterOffset = 3;
    private static final double speedConstant = 0.8;
    int c = 0;
    

    public MoveToGamepiece(
            SwerveSubsystem swerve,
            CoralDetectionSystem coralDetection,
            IntakeSubsystem intake
    ) {
        this.swerve = swerve;
        this.coralDetection = coralDetection;
        this.intake = intake;
        addRequirements(swerve);
    }

    @Override
    public void execute() {
        double coralYaw = coralDetection.getYaw();
        double rotationCorrection = (coralYaw-coralCenterOffset)*-turningConstant;
        swerve.drive(
            new Translation2d(-Math.max(Math.abs(1.5),Math.abs(1.5))*speedConstant,0),
            rotationCorrection,
            false
        );
    }

    @Override
    public boolean isFinished() {
        if (!(coralDetection.isGamepieceDetected())) {
            c++;
        } else {
            c = 0;
        }
        // End if driver releases assist button or intake detects gamepiece in basket
        return c>16 || intake.inBasket();
    }

    @Override
    public void end(boolean interrupted) {
        // Stop the robot
        swerve.drive(new Translation2d(0, 0), 0, true);
    }
}