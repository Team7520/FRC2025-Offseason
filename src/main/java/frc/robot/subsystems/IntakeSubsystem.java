package frc.robot.subsystems;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.MAXMotionConfig.MAXMotionPositionMode;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.IntakeConstants.PivotPosition;

public class IntakeSubsystem extends SubsystemBase {
    // Motors
    private final TalonFX leftIndexer;
    private final TalonFX rightIndexer;
    private final TalonFX intakeRoller;
    private final SparkFlex leftPivot;
    private final SparkFlex rightPivot;
    private final SparkClosedLoopController pivotController;
    private final RelativeEncoder pivotEncoder;


    private final DutyCycleOut duty = new DutyCycleOut(0);

    // --- Speed multipliers (constants for tuning) ---
    private static final double LEFT_INDEXER_MULTIPLIER = 0.35;
    private static final double RIGHT_INDEXER_MULTIPLIER = -0.35;
    private static final double ROLLER_MULTIPLIER = -0.5;
    private static final double PIVOT_MULTIPLIER = 0.5;

    // PID for pivot hold
    private double pivotHoldPosition = 0;
    private static final double JOYSTICK_DEADBAND = 0.05;

    private AnalogInput sensor = new AnalogInput(0);
    private double basketThreshold = 3.75;


    public IntakeSubsystem(int leftIndexerId, int rightIndexerId, int rollerId, int leftPivotId, int rightPivotId) {
        leftIndexer = new TalonFX(leftIndexerId);
        rightIndexer = new TalonFX(rightIndexerId);
        intakeRoller = new TalonFX(rollerId);
    
        leftPivot = new SparkFlex(leftPivotId, MotorType.kBrushless);
        rightPivot = new SparkFlex(rightPivotId, MotorType.kBrushless);
        pivotEncoder = leftPivot.getEncoder();
        pivotController = leftPivot.getClosedLoopController();
    
        // Configure left pivot normally
        SparkFlexConfig pivotConfig = new SparkFlexConfig();
        pivotConfig.smartCurrentLimit(60);
        pivotConfig.idleMode(IdleMode.kBrake);
        pivotConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .p(0.4) // tune
            .i(0)
            .d(0)
            .outputRange(-1, 1);
        
        pivotConfig.smartCurrentLimit(60);
        pivotConfig.idleMode(IdleMode.kBrake);

        pivotConfig.closedLoop.maxMotion
        .maxVelocity(IntakeConstants.MAX_VELOCITY)
        .maxAcceleration(IntakeConstants.MAX_ACCELERATION)
        .allowedClosedLoopError(IntakeConstants.ALLOWABLE_ERROR);

        leftPivot.configure(pivotConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        SparkFlexConfig followerConfig = new SparkFlexConfig();
        followerConfig
            .follow(leftPivot.getDeviceId(), true); // false = not inverted, set true if needed

        rightPivot.configure(followerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        
    }

    public void runIntake(double speed) {
        leftIndexer.setControl(duty.withOutput(speed * LEFT_INDEXER_MULTIPLIER));
        rightIndexer.setControl(duty.withOutput(speed * RIGHT_INDEXER_MULTIPLIER));
        intakeRoller.setControl(duty.withOutput(speed * ROLLER_MULTIPLIER));
    }

    public Command reverseIntake(double speed) {
        return Commands.run(() -> runIntake(speed), this).finallyDo(() -> stopAll());
    }

    public Command intakePiece() {
        return Commands.run(
            () -> runIntake(1), 
            this
        ).until(() -> inBasket()).andThen(() -> stopAll());
    }

    public void runPivot(double speed) {
        if (Math.abs(speed) > JOYSTICK_DEADBAND) {
            leftPivot.set(speed*PIVOT_MULTIPLIER);
        }
    }

    public void stopAll() {
        leftIndexer.setControl(duty.withOutput(0));
        rightIndexer.setControl(duty.withOutput(0));
        intakeRoller.setControl(duty.withOutput(0));
    }

    public void manual(double addRotations) {
        pivotController.setReference(pivotEncoder.getPosition() + addRotations, ControlType.kPosition);
    }

    public void manaulSetPos() {
        pivotEncoder.setPosition(16.754045486450195);
    }

    public void setPivotPosition(Constants.IntakeConstants.PivotPosition position) {
        pivotController.setReference(position.getAngle(), ControlType.kMAXMotionPositionControl);
    }

    public Command setPivotPositionCommand(PivotPosition down) {
        return Commands.runOnce(() -> setPivotPosition(down), this);
    }

    public boolean inBasket() { 
        return sensor.getVoltage() <= basketThreshold;
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Analog Sensor", sensor.getVoltage());
        SmartDashboard.putBoolean("In Basket?", inBasket());
        SmartDashboard.putNumber("Intake Pivot", leftPivot.getEncoder().getPosition());
    }
}
