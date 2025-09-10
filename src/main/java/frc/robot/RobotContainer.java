// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ApriltagConstants;
import frc.robot.Constants.ArmConstants.ArmPositions;
import frc.robot.Constants.ElevatorConstants.ElevatorPosition;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.AlgaePickupCommand;
import frc.robot.commands.BargeCommand;
import frc.robot.commands.CoralPlaceCommand;
import frc.robot.commands.DriveToPoseCommand;
import frc.robot.commands.HandIntakeCommand;
import frc.robot.commands.HighAlgaeCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.L1Command;
import frc.robot.commands.L2Command;
import frc.robot.commands.L2PlaceCommand;
import frc.robot.commands.L3Command;
import frc.robot.commands.L3PlaceCommand;
import frc.robot.commands.L4Command;
import frc.robot.commands.L4PlaceCommand;
import frc.robot.commands.LowAlgaeCommand;
import frc.robot.commands.ManualElevator;
import frc.robot.commands.ManualIntake;
import frc.robot.commands.PickupCoralCommand;
import frc.robot.commands.ReadyToPickupCommand;
import frc.robot.commands.ProcessorAlgae;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import java.io.File;
import java.util.Set;

import swervelib.SwerveInputStream;
//import frc.robot.AprilTagSystem;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer
{

  // Replace with CommandPS4Controller or CommandJoystick if needed
  final         CommandXboxController driverXbox = new CommandXboxController(0);
  final CommandXboxController operatorController = new CommandXboxController(1);

  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem       drivebase  = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
                                                                                "swerve/kraken-jetstream"));

  /**
   * Converts driver input into a field-relative ChassisSpeeds that is controlled by angular velocity.
   */
  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
                                                                () -> driverXbox.getLeftY() * -0.3,
                                                                () -> driverXbox.getLeftX() * -0.3)
                                                            .withControllerRotationAxis(() -> driverXbox.getRightX() * -0.3)
                                                            .deadband(OperatorConstants.DEADBAND)
                                                            .scaleTranslation(0.8)
                                                            .allianceRelativeControl(false);

  /**
   * Clone's the angular velocity input stream and converts it to a fieldRelative input stream.
   */
  SwerveInputStream driveDirectAngle = driveAngularVelocity.copy().withControllerHeadingAxis(driverXbox::getRightX,
                                                                                             driverXbox::getRightY)
                                                           .headingWhile(true);

  /**
   * Clone's the angular velocity input stream and converts it to a robotRelative input stream.
   */
  SwerveInputStream driveRobotOriented = driveAngularVelocity.copy().robotRelative(true)
                                                             .allianceRelativeControl(false);

  SwerveInputStream driveAngularVelocityKeyboard = SwerveInputStream.of(drivebase.getSwerveDrive(),
                                                                        () -> -driverXbox.getLeftY(),
                                                                        () -> -driverXbox.getLeftX())
                                                                    .withControllerRotationAxis(() -> driverXbox.getRawAxis(
                                                                        2))
                                                                    .deadband(OperatorConstants.DEADBAND)
                                                                    .scaleTranslation(0.8)
                                                                    .allianceRelativeControl(true);
  // Derive the heading axis with math!
  SwerveInputStream driveDirectAngleKeyboard     = driveAngularVelocityKeyboard.copy()
                                                                               .withControllerHeadingAxis(() ->
                                                                                                              Math.sin(
                                                                                                                  driverXbox.getRawAxis(
                                                                                                                      2) *
                                                                                                                  Math.PI) *
                                                                                                              (Math.PI *
                                                                                                               2),
                                                                                                          () ->
                                                                                                              Math.cos(
                                                                                                                  driverXbox.getRawAxis(
                                                                                                                      2) *
                                                                                                                  Math.PI) *
                                                                                                              (Math.PI *
                                                                                                               2))
                                                                               .headingWhile(true)
                                                                               .translationHeadingOffset(true)
                                                                               .translationHeadingOffset(Rotation2d.fromDegrees(
                                                                                   0));

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */

  private String mode = "Coral";
  private final ClimberSubsystem climber = new ClimberSubsystem(35);
  private final ArmSubsystem arm = new ArmSubsystem(() -> mode);
  private final IntakeSubsystem intake = new IntakeSubsystem(
        21, // left indexer X44
        22, // right indexer X44
        23, // intake roller X60
        28,  // right pivot
        29   // left pivot
    );
  private final ElevatorSubsystem elevator = new ElevatorSubsystem();
  private final AprilTagSystem aprilTagSystem = new AprilTagSystem();
  private boolean algaePos = false;
  private String coralLevel = "none";
  

  public RobotContainer()
  {
    // Configure the trigger bindings
    configureBindings();
    DriverStation.silenceJoystickConnectionWarning(true);
    NamedCommands.registerCommand("test", Commands.print("I EXIST"));
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary predicate, or via the
   * named factories in {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4}
   * controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight joysticks}.
   */
  
  private void configureBindings()
  {
    operatorController.povUp().onTrue(new InstantCommand(() -> {
        if (mode.equals("Coral")) {
            mode = "Algae";
        } else {
            mode = "Coral";
        }
        System.out.println("Mode switched to: " + mode);
    }));
    
    operatorController.povRight().onTrue(elevator.resetEncoderCommand());

    operatorController.povDown().onTrue(new InstantCommand(() -> intake.manaulSetPos()));

    operatorController.leftTrigger().whileTrue(new InstantCommand(() -> {
      if(!intake.inBasket() && mode.equals("Coral")) {
        new IntakeCommand(intake, operatorController::getLeftTriggerAxis).schedule();
      } else if(arm.algaePos()) {
        new HandIntakeCommand(arm, operatorController::getLeftTriggerAxis).schedule();
      } else {
        intake.setPivotPositionCommand(Constants.IntakeConstants.PivotPosition.UP).schedule();
      }
    }));
    
    


    // B button → intake until piece detected, then hold
  // operatorController.b().whileTrue(
  //   Commands.run(() -> arm.intake(), arm)   // run intake
  //       .until(arm::hasPiece)              // stop if sensor detects piece
  //       .finallyDo(interrupted -> arm.captureHoldFromEncoder()) // hold when finished
  // );

  // A button → eject while held, stop when released
  // operatorController.a()
  //   .whileTrue(Commands.run(arm::eject, arm))
  //   .onFalse(Commands.runOnce(arm::stopOpenLoop, arm));

  // Default command → pivot follows right joystick Y (scaled down)
  arm.setDefaultCommand(
    new RunCommand(
        () -> arm.updatePivotWithJoystick(operatorController.getRightX() * 0.2),
        arm
    )
  );



  intake.setDefaultCommand(
        new ManualIntake(
          intake, 
            () -> operatorController.getRightY()
        )
    );
    
    // elevator
    elevator.setDefaultCommand(
            new ManualElevator(elevator, () -> -operatorController.getLeftY())
        );
    
    
    driverXbox.x()
        .whileTrue(new RunCommand(() -> climber.setPower(0.8), climber))
        .onFalse(new RunCommand(() -> climber.holdPosition(), climber));
    
    driverXbox.y()
        .whileTrue(new RunCommand(() -> climber.setPower(-0.8), climber))
        .onFalse(new RunCommand(() -> climber.holdPosition(), climber));
    //driverXbox.x().onTrue(elevator.moveAndWaitToPosition(ElevatorPosition.L4));

    //operatorController.leftBumper().onTrue()
    
    // operatorController.a().onTrue(elevator.moveToPosition(ElevatorPosition.LOW));

    /* HAND COMMANDS */

    //     COMMENTED OUT TO AVOID INTERFERING WITH TESTING
    
    // //EVERYTHING FOR A
    operatorController.a().onTrue(new InstantCommand(() -> {
      if(!arm.hasPiece()) {
        new AlgaePickupCommand(arm, elevator).andThen(() -> arm.setAlgaePos(true)).schedule();
      } else if(mode.equals("Coral") && arm.hasCoral() && !arm.checkScoreSide()){
        new L1Command(arm,elevator, false).schedule();
      } else if(mode.equals("Coral") && arm.hasCoral() && arm.checkScoreSide()) {
        new L1Command(arm,elevator, true).schedule();
      } else if(mode.equals("Algae") && arm.hasAlgae()) {
        new ProcessorAlgae(arm, elevator).schedule();
      }
    }));

    // //EVERYTHING FOR B
    operatorController.b().onTrue(new InstantCommand(() -> {
      if(!arm.hasPiece() && !arm.checkScoreSide()) {
        new LowAlgaeCommand(arm, elevator, false).andThen(() -> arm.setAlgaePos(true)).schedule();
      } else if(!arm.hasPiece() && arm.checkScoreSide()) {
        new LowAlgaeCommand(arm, elevator, true).andThen(() -> arm.setAlgaePos(true)).schedule();
      } else if(mode.equals("Coral") && arm.hasCoral() && !arm.checkScoreSide()){
        new L2Command(arm,elevator, false).andThen(() -> coralLevel = "L2").schedule();
      } else if(mode.equals("Coral") && arm.hasCoral() && arm.checkScoreSide()) {
        new L2Command(arm,elevator, true).andThen(() -> coralLevel = "L2").schedule();
      }
    }));

    // //EVERYTHING FOR X
    operatorController.x().onTrue(new InstantCommand(() -> {
      if(!arm.hasPiece() && !arm.checkScoreSide()) {
        new HighAlgaeCommand(arm, elevator, false).andThen(() -> arm.setAlgaePos(true)).schedule();
      } else if(!arm.hasPiece() && arm.checkScoreSide()) {
        new HighAlgaeCommand(arm, elevator, true).andThen(() -> arm.setAlgaePos(true)).schedule();
      } else if(mode.equals("Coral") && arm.hasCoral() && !arm.checkScoreSide()){
        new L3Command(arm,elevator, false).andThen(() -> coralLevel = "L3").schedule();
      } else if(mode.equals("Coral") && arm.hasCoral() && arm.checkScoreSide()) {
        new L3Command(arm,elevator, true).andThen(() -> coralLevel = "L3").schedule();
      }}));

    // //EVERYTHING FOR Y
    operatorController.y().onTrue(new InstantCommand(() -> {
      if(mode.equals("Coral") && arm.hasCoral() && !arm.checkScoreSide()){
         new L4Command(arm,elevator, false).andThen(() -> coralLevel = "L4").schedule();
      } else if(mode.equals("Coral") && arm.hasCoral() && arm.checkScoreSide()) {
         new L4Command(arm,elevator, true).andThen(() -> coralLevel = "L4").schedule();
      }else if(mode.equals("Algae") && arm.hasAlgae() && !arm.checkScoreSide()) {
         new BargeCommand(arm, elevator, false).schedule();
      }else if(mode.equals("Algae") && arm.hasAlgae() && arm.checkScoreSide()) {
        new BargeCommand(arm, elevator, true).schedule();
      }
    }));

    operatorController.leftBumper().onTrue(new InstantCommand(() -> {
      if(arm.algaePos()) {
        arm.moveToPosition(Constants.ArmConstants.ArmPositions.DEFAULT).andThen(() -> arm.setAlgaePos(false)).schedule();
      } else if(intake.inBasket()) {
        new ReadyToPickupCommand(arm, elevator).andThen(new PickupCoralCommand(arm, elevator)).schedule();
      } else {
        new ReadyToPickupCommand(arm, elevator).schedule();
      }
    }));
      
    // operatorController.rightTrigger().onTrue(new InstantCommand(() -> {
    //   if(!arm.checkScoreSide()) {
    //       arm.moveToPosition(ArmPositions.SCORE).schedule();;
    //   } else {
    //     arm.moveToPosition(ArmPositions.OPP_SCORE).schedule();;
    //   }
    // }));
    
    operatorController.rightBumper().onTrue(arm.changeScoreSide());

    operatorController.rightTrigger().onTrue(new InstantCommand(() -> {
      if(!arm.checkScoreSide()) {
        if(coralLevel.equals("L4")) {
          new L4PlaceCommand(arm, elevator, false).andThen(() -> coralLevel = "none").schedule();
        } else if(coralLevel.equals("L3")) {
          new L3PlaceCommand(arm, elevator, false).andThen(() -> coralLevel = "none").schedule();
        } else if(coralLevel.equals("L2")) {
          new L2PlaceCommand(arm, elevator, false).andThen(() -> coralLevel = "none").schedule();
        } else {
          arm.ejectPiece(0.5).schedule();
        }  
      } else if(arm.checkScoreSide()) {
        if(coralLevel.equals("L4")) {
          new L4PlaceCommand(arm, elevator, true).andThen(() -> coralLevel = "none").schedule();
        } else if(coralLevel.equals("L3")) {
          new L3PlaceCommand(arm, elevator, true).andThen(() -> coralLevel = "none").schedule();
        } else if(coralLevel.equals("L2")) {
          new L2PlaceCommand(arm, elevator, true).andThen(() -> coralLevel = "none").schedule();
        } else {
          arm.ejectPiece(0.5).schedule();
        }  
      }
    }));


  //   driverXbox.leftBumper().whileTrue(new InstantCommand(() -> {
  //     Pose2d tagPose = aprilTagSystem.getClosestTagPose();
  //     System.out.println("LEFT BUMPER PRESSED");
  //     if (tagPose != null) {
  //         Pose2d offsetPose = aprilTagSystem.getOffsetPose(tagPose, ApriltagConstants.xOffsetLeft, ApriltagConstants.yOffsetLeft);
  //         System.out.println("TARGET POSE:");
  //         System.out.println(offsetPose);
  //         Pose2d robotPose = drivebase.getPose();
  
  //         new DriveToPoseCommand(
  //             robotPose,
  //             offsetPose,
  //             driverXbox::getLeftX,  // x input
  //             driverXbox::getLeftY   // y input (forward/back)
  //         ).schedule();
  //     }
  // }));
  driverXbox.leftBumper().whileTrue(
    Commands.defer(() -> {
        Pose2d tagPose = aprilTagSystem.getClosestTagPose();
        if (tagPose == null) {
            //System.out.println("No valid AprilTag found.");
            return new InstantCommand(); // ends immediately
        }

        Pose2d offsetPose = aprilTagSystem.getOffsetPose(
            tagPose,
            ApriltagConstants.xOffsetLeft,
            ApriltagConstants.yOffsetLeft
        );

        System.out.println("Driving to offset pose: " + offsetPose);

        return new DriveToPoseCommand(
            drivebase.getPose(),
            offsetPose,
            driverXbox::getLeftX,
            driverXbox::getLeftY
        );
    }, Set.of(drivebase)) // requirements
  );

  driverXbox.rightBumper().whileTrue(
    Commands.defer(() -> {
        Pose2d tagPose = aprilTagSystem.getClosestTagPose();
        if (tagPose == null) {
            System.out.println("No tag detected (right bumper).");
            return new InstantCommand();
        }

        Pose2d offsetPose = aprilTagSystem.getOffsetPose(
            tagPose,
            ApriltagConstants.xOffsetRight,
            ApriltagConstants.yOffsetRight
        );

        return new DriveToPoseCommand(
            drivebase.getPose(),
            offsetPose,
            driverXbox::getLeftX,
            driverXbox::getLeftY
        );
    }, Set.of(drivebase))


  );


    Command driveFieldOrientedDirectAngle      = drivebase.driveFieldOriented(driveDirectAngle);
    Command driveFieldOrientedAngularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);
    Command driveRobotOrientedAngularVelocity  = drivebase.driveFieldOriented(driveRobotOriented);
    Command driveSetpointGen = drivebase.driveWithSetpointGeneratorFieldRelative(
        driveDirectAngle);
    Command driveFieldOrientedDirectAngleKeyboard      = drivebase.driveFieldOriented(driveDirectAngleKeyboard);
    Command driveFieldOrientedAngularVelocityKeyboard = drivebase.driveFieldOriented(driveAngularVelocityKeyboard);
    Command driveSetpointGenKeyboard = drivebase.driveWithSetpointGeneratorFieldRelative(
        driveDirectAngleKeyboard);

    if (RobotBase.isSimulation())
    {
      drivebase.setDefaultCommand(driveFieldOrientedDirectAngleKeyboard);
    } else
    {
      drivebase.setDefaultCommand(driveFieldOrientedAngularVelocity);
    }

    if (Robot.isSimulation())
    {
      Pose2d target = new Pose2d(new Translation2d(1, 4),
                                 Rotation2d.fromDegrees(90));
      //drivebase.getSwerveDrive().field.getObject("targetPose").setPose(target);
      driveDirectAngleKeyboard.driveToPose(() -> target,
                                           new ProfiledPIDController(5,
                                                                     0,
                                                                     0,
                                                                     new Constraints(5, 2)),
                                           new ProfiledPIDController(5,
                                                                     0,
                                                                     0,
                                                                     new Constraints(Units.degreesToRadians(360),
                                                                                     Units.degreesToRadians(180))
                                           ));
      driverXbox.start().onTrue(Commands.runOnce(() -> drivebase.resetOdometry(new Pose2d(3, 3, new Rotation2d()))));
      driverXbox.button(1).whileTrue(drivebase.sysIdDriveMotorCommand());
      driverXbox.button(2).whileTrue(Commands.runEnd(() -> driveDirectAngleKeyboard.driveToPoseEnabled(true),
                                                     () -> driveDirectAngleKeyboard.driveToPoseEnabled(false)));

//      driverXbox.b().whileTrue(
//          drivebase.driveToPose(
//              new Pose2d(new Translation2d(4, 4), Rotation2d.fromDegrees(0)))
//                              );

    }
    if (DriverStation.isTest())
    {
      drivebase.setDefaultCommand(driveFieldOrientedAngularVelocity); // Overrides drive command above!

      driverXbox.x().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
      driverXbox.y().whileTrue(drivebase.driveToDistanceCommand(1.0, 0.2));
      driverXbox.start().onTrue((Commands.runOnce(drivebase::zeroGyro)));
      driverXbox.back().whileTrue(drivebase.centerModulesCommand());
      driverXbox.leftBumper().onTrue(Commands.none());
      driverXbox.rightBumper().onTrue(Commands.none());
    } else
    {
      driverXbox.a().onTrue((Commands.runOnce(drivebase::zeroGyro)));
      //driverXbox.x().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
      // driverXbox.x().onTrue(Commands.runOnce(drivebase::addFakeVisionReading));
      // driverXbox.start().whileTrue(Commands.none());
      // driverXbox.back().whileTrue(Commands.none());
      // driverXbox.leftBumper().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
      // driverXbox.rightBumper().onTrue(Commands.none());
    }

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand()
  {
    // An example command will be run in autonomous
    return drivebase.getAutonomousCommand("New Auto");
  }

  public void setMotorBrake(boolean brake)
  {
    drivebase.setMotorBrake(brake);
  }
}
