package frc.robot.subsystems;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimberSubsystem extends SubsystemBase {
    private final SparkFlex climberMotor;
    private final SparkClosedLoopController closedLoop;
    private double targetPosition;

    public ClimberSubsystem(int motorID) {
        climberMotor = new SparkFlex(motorID, MotorType.kBrushless);

        SparkFlexConfig climberConfig = new SparkFlexConfig();
        climberConfig.smartCurrentLimit(60);
        climberConfig.idleMode(IdleMode.kBrake);
        climberConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .p(0.5)  // tune
            .i(0)
            .d(0)
            .outputRange(-1, 1);

        climberMotor.configure(climberConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        closedLoop = climberMotor.getClosedLoopController();

        targetPosition = climberMotor.getEncoder().getPosition();
    }

    public void setPower(double power) {
        climberMotor.set(power);
        targetPosition = climberMotor.getEncoder().getPosition();
    }

    public void holdPosition() {
        closedLoop.setReference(targetPosition, ControlType.kPosition);
    }

    public void stop() {
        climberMotor.stopMotor();
    }
}
