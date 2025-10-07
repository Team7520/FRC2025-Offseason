package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.CoralDetectionSystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Rotation2d;

/**
 * Assisted drive: driver controls the robot, but vision steers left/right to line up with the gamepiece.
 * Exits when intake detects a gamepiece in the basket.
 */
public class VisionAssistedDriveCommand extends Command {

    private final SwerveSubsystem swerve;
    private final CoralDetectionSystem coralDetection;
    private final IntakeSubsystem intake;
    private final CommandXboxController driverController;

    private static final double turningConstant = 0.03;
    private static final double speedConstant = 0.8;
    private static final double coralCenterOffset = 3;

    public VisionAssistedDriveCommand(
            SwerveSubsystem swerve,
            CoralDetectionSystem coralDetection,
            IntakeSubsystem intake,
            CommandXboxController driverController
    ) {
        this.swerve = swerve;
        this.coralDetection = coralDetection;
        this.intake = intake;
        this.driverController = driverController;

        addRequirements(swerve);
    }

    @Override
    public void execute() {
        // Driver joystick values (field-oriented)
        double driverY = -driverController.getLeftY(); // Forward/backward
        double driverX = -driverController.getLeftX(); // Left/right
        double driverRotation = driverController.getRightX(); // Rotation

        // Default: no steering assist
        // double xFieldCorrection = 0.0;
        // double yFieldCorrection = 0.0;
        double rotationCorrection = 0;

        if (driverController.getLeftTriggerAxis() > 0.5
            && coralDetection.isGamepieceDetected()
            && !intake.inBasket()) {
            // double xErrorRobot = coralDetection.robotCoralTranslation.getY();
            // double yErrorRobot = coralDetection.robotCoralTranslation.getX();
            double coralYaw = coralDetection.getYaw();
            rotationCorrection = (coralYaw-coralCenterOffset)*-turningConstant;
            
            // double yErrorRobot = coralDetection.getEstimatedX(); // robot-relative X offset
            // double xErrorRobot = coralDetection.getEstimatedY(); // robot-relative Y offset

            // Only apply correction if both are valid (not -1)
            // if (xErrorRobot != -1 && yErrorRobot != -1) {
            //     //double robotHeading = swerve.getPose().getRotation().getRadians(); // Your swerve subsystem must provide the field heading

            //     // Transform robot-relative vision error to field-relative
            //     // xFieldCorrection = (kSteer * ( xErrorRobot * Math.cos(robotHeading) - yErrorRobot * Math.sin(robotHeading) )) * Math.abs(driverY);
            //     // yFieldCorrection = (kSteer * ( xErrorRobot * Math.sin(robotHeading) + yErrorRobot * Math.cos(robotHeading) )) * Math.abs(driverY);
            //     yFieldCorrection = kSteer * yErrorRobot * Math.abs(driverY);
            //     xFieldCorrection = kSteer * xErrorRobot * Math.abs(driverX);


            // }
            //System.out.println("ASSISTED DRIVE: y:"+yFieldCorrection+"  x:"+xFieldCorrection+ " XE:"+xErrorRobot+" YE:"+yErrorRobot);
            swerve.drive(
                new Translation2d(-Math.max(Math.abs(driverX),Math.abs(driverY))*speedConstant,0),
                driverRotation + rotationCorrection,
                false
            );
            //System.out.println("ASSISTED DRIVING "+ (Math.abs(driverX)+Math.abs(driverY))*0.5);

        } else {

            swerve.drive(
                new Translation2d(driverY, driverX),
                driverRotation + rotationCorrection,
                true
            );
        }
    }

    @Override
    public boolean isFinished() {
        // End if driver releases assist button or intake detects gamepiece in basket
        return !(driverController.getLeftTriggerAxis() > 0.5)
            || intake.inBasket();
    }

    @Override
    public void end(boolean interrupted) {
        // Stop the robot
        swerve.drive(new Translation2d(0, 0), 0, true);
    }
}