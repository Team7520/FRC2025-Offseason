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
import frc.robot.Constants.ArmConstants.ArmPositions;
import frc.robot.Constants.ElevatorConstants.ElevatorPosition;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.AlgaePickupCommand;
import frc.robot.commands.DriveToPoseCommand;
import frc.robot.commands.HighAlgaeCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.L1Command;
import frc.robot.commands.L2Command;
import frc.robot.commands.L3Command;
import frc.robot.commands.L4Command;
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
  private final ClimberSubsystem climber = new ClimberSubsystem(35);
  private final ArmSubsystem arm = new ArmSubsystem();
  private final IntakeSubsystem intake = new IntakeSubsystem(
        21, // left indexer X44
        22, // right indexer X44
        23, // intake roller X60
        28,  // right pivot
        29   // left pivot
    );
  private final ElevatorSubsystem elevator = new ElevatorSubsystem();
  private final AprilTagSystem aprilTagSystem = new AprilTagSystem();
  

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
  private String mode = "Coral";
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
    operatorController.leftTrigger().whileTrue(new IntakeCommand(intake, arm, () -> mode));
    


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



  // intake.setDefaultCommand(
  //       new ManualIntake(
  //         intake, 
  //           () -> -operatorController.getRightY()
  //       )
  //   );
    
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

    //operatorController.leftBumper().onTrue()
    
    // operatorController.a().onTrue(elevator.moveToPosition(ElevatorPosition.LOW));

    /* HAND COMMANDS */

    //     COMMENTED OUT TO AVOID INTERFERING WITH TESTING
    
    // //EVERYTHING FOR A
    operatorController.a().onTrue(new InstantCommand(() -> {
      if(!arm.hasPiece()) {
        new AlgaePickupCommand(arm, elevator).schedule();
      } else if(mode.equals("Coral") && arm.hasPiece()){
        new L1Command(arm,elevator).schedule();
      }else if(mode.equals("Algae") && arm.hasPiece()) {
        new ProcessorAlgae(arm, elevator).schedule();
      }
    }));

    // //EVERYTHING FOR B
    operatorController.b().onTrue(new InstantCommand(() -> {
      if(((mode.equals("Coral") || mode.equals("Algae")) && !arm.hasPiece())) {
        new LowAlgaeCommand(arm, elevator).schedule();
      } else if(mode.equals("Coral") && arm.hasPiece()){
        new L2Command(arm,elevator).schedule();
    }}));

    // //EVERYTHING FOR X
    operatorController.x().onTrue(new InstantCommand(() -> {
      if(((mode.equals("Coral") || mode.equals("Algae")) && !arm.hasPiece())) {
        new HighAlgaeCommand(arm, elevator).schedule();
      } else if(mode.equals("Coral") && arm.hasPiece()){
        new L3Command(arm,elevator).schedule();
    }}));

    // //EVERYTHING FOR Y
    operatorController.y().onTrue(new InstantCommand(() -> {
      new L4Command(arm,elevator).schedule();
      // if(mode.equals("Coral") && arm.hasCoral()){
      //   arm.moveToPosition(ArmPositions.L4).schedule(); //replace with a command that includes elevator later L4
      // } /*else if(mode.equals("Algae") && arm.hasAlgae()) {
      //   arm.moveToPosition(ArmPositions.BARGE).schedule();; //replace with a comman that includes elevator BARGE
      // }*/
    }));

    operatorController.leftBumper().onTrue(new InstantCommand(() -> {
      if(true/*coral basket sensor is true */) {
        new ReadyToPickupCommand(arm, elevator).andThen(new PickupCoralCommand(arm, elevator)).schedule();;
      } else {
        new ReadyToPickupCommand(arm, elevator).schedule();;
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

    operatorController.rightTrigger().whileTrue(arm.placeCoralCommand(0.5));


    double yOffsetLeft = 0.17;
    double xOffsetLeft = 0.65;
    driverXbox.leftBumper().onTrue(new InstantCommand(() -> {
      Pose2d tagPose = aprilTagSystem.getClosestTagPose();
      System.out.println("LEFT BUMPER PRESSED");
      if (tagPose != null) {
          Pose2d offsetPose = aprilTagSystem.getOffsetPose(tagPose, yOffsetLeft, -xOffsetLeft);
          System.out.println("TARGET POSE:");
          System.out.println(offsetPose);
          Pose2d robotPose = drivebase.getPose();
  
          new DriveToPoseCommand(
              robotPose,
              offsetPose,
              driverXbox::getLeftX,  // x input
              driverXbox::getLeftY   // y input (forward/back)
          ).schedule();
      }
  }));
  double yOffsetRight = 0.16;
  double xOffsetRight = 0.65;
  driverXbox.rightBumper().onTrue(new InstantCommand(() -> {
    Pose2d tagPose = aprilTagSystem.getClosestTagPose();
    if (tagPose != null) {
        Pose2d offsetPose = aprilTagSystem.getOffsetPose(tagPose, xOffsetRight, yOffsetRight);
        Pose2d robotPose = drivebase.getPose();

        new DriveToPoseCommand(
            robotPose,
            offsetPose,
            driverXbox::getLeftX,  // x input
            driverXbox::getLeftY   // y input (forward/back)
        ).schedule();
    }}));


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
