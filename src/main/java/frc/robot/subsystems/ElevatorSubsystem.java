package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.MotorArrangementValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorSubsystem extends SubsystemBase {
    private final TalonFXS leftMotor;
    private final TalonFXS rightMotor;

    private final DutyCycleOut dutyCycleOut = new DutyCycleOut(0);
    private final PositionDutyCycle positionDutyCycle = new PositionDutyCycle(0);

    public ElevatorSubsystem(int leftID, int rightID) {
        leftMotor = new TalonFXS(leftID);
        rightMotor = new TalonFXS(rightID);

        TalonFXSConfiguration config = new TalonFXSConfiguration();
        config.Commutation.MotorArrangement = MotorArrangementValue.Minion_JST;
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.Slot0.kP = 0.5; // tune PID
        config.Slot0.kI = 0.0;
        config.Slot0.kD = 0.0;

        leftMotor.getConfigurator().apply(config);
        rightMotor.getConfigurator().apply(config);

        leftMotor.setInverted(true);
        rightMotor.setInverted(false);
    }

    public void setPower(double power) {
        leftMotor.setControl(dutyCycleOut.withOutput(power));
        rightMotor.setControl(dutyCycleOut.withOutput(power));
    }
    public void setPosition(double rotations) {
        leftMotor.setControl(positionDutyCycle.withPosition(rotations));
        rightMotor.setControl(positionDutyCycle.withPosition(rotations));
    }

    public void stop() {
        setPower(0);
    }

    public double getPosition() {
        return (leftMotor.getPosition().getValueAsDouble() + rightMotor.getPosition().getValueAsDouble()) / 2.0;
    }
}

       
        