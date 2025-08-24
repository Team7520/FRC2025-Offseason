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
    // Roller
    private final TalonFX roller;
    private final DutyCycleOut rollerDuty = new DutyCycleOut(0);
    private final PositionDutyCycle rollerPosReq = new PositionDutyCycle(0);

    // Piece detection
    private final LaserCan laser;
    private double pieceThresholdMM = 0;

    // Pivot
    private final TalonFX pivot;
    private final DutyCycleOut pivotDuty = new DutyCycleOut(0);
    private final PositionDutyCycle pivotPosReq = new PositionDutyCycle(0);

    private double holdPivotRot = 0.0;
    private boolean pivotHolding = false;

    private double holdRollerRot = 0.0;
    private boolean rollerHolding = false;

    public ArmSubsystem(int rollerCanId, int laserCanId, int pivotCanId) {
        roller = new TalonFX(rollerCanId);
        laser = new LaserCan(laserCanId);

        // Roller config
        TalonFXConfiguration cfg = new TalonFXConfiguration();
        cfg.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        cfg.Voltage.PeakForwardVoltage = 12.0;
        cfg.Voltage.PeakReverseVoltage = -12.0;
        cfg.CurrentLimits.SupplyCurrentLimitEnable = true;
        cfg.CurrentLimits.SupplyCurrentLimit = 35.0;
        cfg.Feedback.SensorToMechanismRatio = 1.0;
        cfg.Slot0.kP = 0.35;   // tune
        roller.getConfigurator().apply(cfg);

        // Pivot config
        pivot = new TalonFX(pivotCanId);
        TalonFXConfiguration pivotCfg = new TalonFXConfiguration();
        pivotCfg.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        pivotCfg.Voltage.PeakForwardVoltage = 12.0;
        pivotCfg.Voltage.PeakReverseVoltage = -12.0;
        pivotCfg.CurrentLimits.SupplyCurrentLimitEnable = true;
        pivotCfg.CurrentLimits.SupplyCurrentLimit = 40.0;
        pivotCfg.Feedback.SensorToMechanismRatio = 1.0; // adjust
        pivotCfg.Slot0.kP = 0.4; // tune
        pivot.getConfigurator().apply(pivotCfg);
    }

    // Roller control
    public void intake() {
        roller.setControl(rollerDuty.withOutput(-0.4));
        rollerHolding = false;
    }

    public void eject() {
        roller.setControl(rollerDuty.withOutput(0.8));
        rollerHolding = false;
    }

    public void stopOpenLoop() {
        roller.setControl(rollerDuty.withOutput(0.0));
    }

    public boolean hasPiece() {
        double mm = laser.getMeasurement().distance_mm;
        return mm < pieceThresholdMM;
    }

    public void setPieceThresholdMM(double mm) {
        pieceThresholdMM = mm;
    }

    public void captureHoldFromEncoder() {
        holdRollerRot = roller.getPosition().getValueAsDouble();
        roller.setControl(rollerPosReq.withPosition(holdRollerRot));
        rollerHolding = true;
    }

    public void applyHoldRoller() {
        if (rollerHolding) {
            roller.setControl(rollerPosReq.withPosition(holdRollerRot));
        }
    }

    public void releaseHoldRoller() {
        stopOpenLoop();
        rollerHolding = false;
    }

    // Pivot control
    public void setPivotManual(double percent) {
        pivot.setControl(pivotDuty.withOutput(percent));
        holdPivotRot = pivot.getPosition().getValueAsDouble();
        pivotHolding = false;
    }

    public void startHoldPivot() {
        holdPivotRot = pivot.getPosition().getValueAsDouble();
        pivot.setControl(pivotPosReq.withPosition(holdPivotRot));
        pivotHolding = true;
    }

    public void applyHoldPivot() {
        if (pivotHolding) {
            pivot.setControl(pivotPosReq.withPosition(holdPivotRot));
        }
    }

    public void releaseHoldPivot() {
        pivot.setControl(pivotDuty.withOutput(0.0));
        pivotHolding = false;
    }

    public void updatePivotWithJoystick(double input) {
        if (Math.abs(input) > 0.05) {
            setPivotManual(input);
        } else {
            if (!pivotHolding) {
                startHoldPivot();
            } else {
                applyHoldPivot();
            }
        }
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("hasPiece", hasPiece());
        SmartDashboard.putNumber("LaserMM", laser.getMeasurement().distance_mm);
        SmartDashboard.putNumber("PivotPos", pivot.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("RollerPos", roller.getPosition().getValueAsDouble());
    }
}
