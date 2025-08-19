package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import au.grapplerobotics.LaserCan;

public class ArmSubsystem extends SubsystemBase {
    private final TalonFX roller;
    private final LaserCan laser;

    private final DutyCycleOut duty = new DutyCycleOut(0);

    private final PositionDutyCycle positionReq = new PositionDutyCycle(0);

    private double holdPositionRot = 0.0;
    private double pieceThresholdMM = 4;

    public ArmSubsystem(int rollerCanId, int laserCanId) {
        roller = new TalonFX(rollerCanId);
        laser = new LaserCan(laserCanId);

        TalonFXConfiguration cfg = new TalonFXConfiguration();

        cfg.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        cfg.Voltage.PeakForwardVoltage = 12.0;
        cfg.Voltage.PeakReverseVoltage = -12.0;

        cfg.CurrentLimits.SupplyCurrentLimitEnable = true;
        cfg.CurrentLimits.SupplyCurrentLimit = 35.0;

        cfg.Feedback.SensorToMechanismRatio = 1.0;

        cfg.Slot0.kP = 0.35;   // tune
        cfg.Slot0.kI = 0.0;
        cfg.Slot0.kD = 0.0;
        cfg.Slot0.kS = 0.0;

        roller.getConfigurator().apply(cfg);
    }

    public void intake() {
        roller.setControl(duty.withOutput(-0.2));
    }

    public void eject() {
        roller.setControl(duty.withOutput(0.8));
    }

    public void stopOpenLoop() {
        roller.setControl(duty.withOutput(0.0));
    }

    public boolean hasPiece() {
        double mm = laser.getMeasurement().distance_mm;
        return mm < pieceThresholdMM;
    }

    public void setPieceThresholdMM(double mm) {
        pieceThresholdMM = mm;
    }

    public void captureHoldFromEncoder() {
        holdPositionRot = roller.getPosition().getValueAsDouble();
        roller.setControl(positionReq.withPosition(holdPositionRot));
    }

    public void applyHold() {
        roller.setControl(positionReq.withPosition(holdPositionRot));
    }

    public void releaseHold() {
        stopOpenLoop();
    }

    public void setPosition_kP(double kP) {
        TalonFXConfiguration cfg = new TalonFXConfiguration();
        cfg.Slot0.kP = kP;
        roller.getConfigurator().apply(cfg);
    }
    public void setPosition_kI(double kI) {
        TalonFXConfiguration cfg = new TalonFXConfiguration();
        cfg.Slot0.kI = kI;
        roller.getConfigurator().apply(cfg);
    }
    public void setPosition_kD(double kD) {
        TalonFXConfiguration cfg = new TalonFXConfiguration();
        cfg.Slot0.kD = kD;
        roller.getConfigurator().apply(cfg);
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("hasPiece", hasPiece());
        SmartDashboard.putNumber("LaserMM",laser.getMeasurement().distance_mm);
    }
}
