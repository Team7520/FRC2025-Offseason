package frc.robot.subsystems;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
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

    private final DutyCycleOut duty = new DutyCycleOut(0);

    // --- Speed multipliers (constants for tuning) ---
    private static final double LEFT_INDEXER_MULTIPLIER = 0.5;
    private static final double RIGHT_INDEXER_MULTIPLIER = -0.5;
    private static final double ROLLER_MULTIPLIER = -0.5;
    private static final double PIVOT_MULTIPLIER = 0.2;

    // PID for pivot hold
    private final PIDController pivotPID = new PIDController(0.05, 0.0, 0.0); // tune kP!
    private double pivotHoldPosition = 0;
    private static final double JOYSTICK_DEADBAND = 0.05;

    public IntakeSubsystem(int leftIndexerId, int rightIndexerId, int rollerId, int pivotLeaderId, int pivotFollowerId) {
        leftIndexer = new TalonFX(leftIndexerId);
        rightIndexer = new TalonFX(rightIndexerId);
        intakeRoller = new TalonFX(rollerId);
        leftPivot = new SparkFlex(pivotLeaderId, MotorType.kBrushless);
        rightPivot= new SparkFlex(pivotFollowerId, MotorType.kBrushless);
        SparkFlexConfig climberConfig = new SparkFlexConfig();
        climberConfig.smartCurrentLimit(60);
        climberConfig.idleMode(IdleMode.kBrake);
        climberConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .p(0.5)  // tune
            .i(0)
            .d(0)
            .outputRange(-1, 1);

        leftPivot.configure(climberConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        rightPivot.configure(climberConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    }

    public void runIntake(double speed) {
        leftIndexer.setControl(duty.withOutput(speed * LEFT_INDEXER_MULTIPLIER));
        rightIndexer.setControl(duty.withOutput(speed * RIGHT_INDEXER_MULTIPLIER));
        intakeRoller.setControl(duty.withOutput(speed * ROLLER_MULTIPLIER));
    }

    /** Run only the pivot (manual + PID hold) */
    public void runPivot(double speed) {
        if (Math.abs(speed) > JOYSTICK_DEADBAND) {
            leftPivot.set(speed*PIVOT_MULTIPLIER);
            rightPivot.set(speed*PIVOT_MULTIPLIER);
        } else {
            // // Hold with PID
            // double currentPosition = pivotLeader.getPosition().getValueAsDouble();
            // double output = pivotPID.calculate(currentPosition, pivotHoldPosition);
            // pivotLeader.setControl(duty.withOutput(output));
        }
    }

    public void stopAll() {
        leftIndexer.setControl(duty.withOutput(0));
        rightIndexer.setControl(duty.withOutput(0));
        intakeRoller.setControl(duty.withOutput(0));
        // pivotLeader.setControl(duty.withOutput(0));
    }
}
