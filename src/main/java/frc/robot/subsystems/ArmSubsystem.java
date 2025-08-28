package frc.robot.subsystems;

import javax.swing.text.Position;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.SparkBase.ControlType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.ArmConstants.ArmPositions;
import frc.robot.Constants.ElevatorConstants.ElevatorPosition;
import au.grapplerobotics.LaserCan;

public class ArmSubsystem extends SubsystemBase {
    // Roller
    private final TalonFX roller;
    private final DutyCycleOut rollerDuty = new DutyCycleOut(0);
    private final PositionDutyCycle rollerPosReq = new PositionDutyCycle(0);
    private final CANcoder encoder;
    private final MotionMagicVoltage motionMagic;
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
    private double armPosition;
    public static double kG = 0;

    public ArmSubsystem() {
        roller = new TalonFX(ArmConstants.ROLLER_CAN_ID);
        laser = new LaserCan(ArmConstants.LASER_CAN_ID);
        encoder = new CANcoder(61);
        


        // Roller config
        TalonFXConfiguration cfg = new TalonFXConfiguration();
        cfg.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        cfg.Voltage.PeakForwardVoltage = 12.0;
        cfg.Voltage.PeakReverseVoltage = -12.0;
        cfg.CurrentLimits.SupplyCurrentLimitEnable = true;
        cfg.CurrentLimits.SupplyCurrentLimit = 35.0;
        cfg.Feedback.SensorToMechanismRatio = 1.0;
        cfg.Slot0.kP = 0.35;   // tune
        // cfg.MotionMagic.MotionMagicCruiseVelocity = ArmConstants.MAX_VELOCITY;
        // cfg.MotionMagic.MotionMagicAcceleration = ArmConstants.MAX_ACCELERATION;
        // cfg.MotionMagic.MotionMagicJerk = ArmConstants.MAX_JERK;
        roller.getConfigurator().apply(cfg);

        // Pivot config
        pivot = new TalonFX(ArmConstants.PIVOT_CAN_ID);
        TalonFXConfiguration pivotCfg = new TalonFXConfiguration();
        pivotCfg.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        pivotCfg.Voltage.PeakForwardVoltage = 12.0;
        pivotCfg.Voltage.PeakReverseVoltage = -12.0;
        pivotCfg.CurrentLimits.SupplyCurrentLimitEnable = true;
        pivotCfg.CurrentLimits.SupplyCurrentLimit = 40.0;
        pivotCfg.Feedback.SensorToMechanismRatio = 1.0; // adjust
        pivotCfg.Slot0.kP = 1.31;
        pivotCfg.MotionMagic.MotionMagicCruiseVelocity = ArmConstants.MAX_VELOCITY;
        pivotCfg.MotionMagic.MotionMagicAcceleration = ArmConstants.MAX_ACCELERATION;
        pivotCfg.MotionMagic.MotionMagicJerk = ArmConstants.MAX_JERK;
        pivotCfg.Feedback.FeedbackRemoteSensorID = encoder.getDeviceID();
        pivotCfg.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;  
        pivot.getConfigurator().apply(pivotCfg);

        motionMagic = new MotionMagicVoltage(0);
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
        double mm = 100;
        if (laser.getMeasurement()!=null){
            mm = laser.getMeasurement().distance_mm;}
            
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

    public void setHoldPosition(double rotations) {
        holdPivotRot = rotations;
    }

    // Pivot control
    public void setPivotManual(double percent) {
        pivot.setControl(pivotDuty.withOutput(percent));
        holdPivotRot = pivot.getPosition().getValueAsDouble();
        pivotHolding = false;
    }

    public void startHoldPivot() {
        holdPivotRot = encoder.getPosition().getValueAsDouble();
        pivot.setControl(pivotPosReq.withPosition(holdPivotRot));
        pivotHolding = true;
    }

    public void applyHoldPivot() {
        if (/*pivotHolding*/ true) {
            //pivot.setControl(pivotPosReq.withPosition(holdPivotRot));
            double theta = pivotAngleRad(); 
            double ff = kG * Math.cos(theta);
            // pivot.setControl(pivotDuty.withOutput(ff));
            pivot.setControl(pivotPosReq.withPosition(holdPivotRot).withFeedForward(ff));
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

    public Command moveToPosition(ArmPositions position) {
        return Commands.run(() -> setPosition(position), this).until(() -> atTarget(position)).andThen(() -> startHoldPivot());
    }
    
    private boolean atTarget(ArmPositions position) {
        double current = encoder.getPosition().getValueAsDouble();
        double error = Math.abs(position.getPosition() - current);
        return error < 0.02;
    }

    private double pivotAngleRad() {
        double absRot = encoder.getAbsolutePosition().getValueAsDouble(); // 0..1 rotations
        double rotFromHoriz = absRot - ArmConstants.PIVOT_HORIZONTAL_OFFSET_ROT;
        return rotFromHoriz * 2.0 * Math.PI; // radians
    }
    

    public void setPosition(ArmConstants.ArmPositions position) {
        armPosition = position.getPosition();
        pivot.setControl(pivotPosReq.withPosition(armPosition));
    }
    

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("hasPiece", hasPiece());
        //SmartDashboard.putNumber("LaserMM", laser.getMeasurement().distance_mm);
        SmartDashboard.putNumber("PivotPos", pivot.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("RollerPos", roller.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("Encoder Value", encoder.getAbsolutePosition().getValueAsDouble());
        SmartDashboard.putBoolean("Is encoder working", encoder.isConnected());
        SmartDashboard.putNumber("Hold Position", holdPivotRot);
    }
}
