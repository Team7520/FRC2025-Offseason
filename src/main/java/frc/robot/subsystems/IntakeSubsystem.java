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
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

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
    private static final double LEFT_INDEXER_MULTIPLIER = 0.5;
    private static final double RIGHT_INDEXER_MULTIPLIER = -0.5;
    private static final double ROLLER_MULTIPLIER = -0.5;
    private static final double PIVOT_MULTIPLIER = 0.2;

    // PID for pivot hold
    private double pivotHoldPosition = 0;
    private static final double JOYSTICK_DEADBAND = 0.05;

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
            .p(0.5) // tune
            .i(0)
            .d(0)
            .outputRange(-1, 1);
        
        pivotConfig.smartCurrentLimit(60);
        pivotConfig.idleMode(IdleMode.kBrake);
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
}
