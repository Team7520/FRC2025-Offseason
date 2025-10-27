package frc.robot.subsystems;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import javax.swing.text.Position;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.OpenLoopRampsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.SparkBase.ControlType;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
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
    private double coralThresholdMM = 3;
    private double algaeThresholdMM = 20;


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
    private boolean sideChange = false;
    private boolean algaePos = false;
    private BooleanSupplier modeSupplier;
    private int trust = 0;
    private Boolean canTrust = false;
    private int trustStrictness = 12;

    public ArmSubsystem(BooleanSupplier modeSupplier) {
        roller = new TalonFX(ArmConstants.ROLLER_CAN_ID);
        laser = new LaserCan(ArmConstants.LASER_CAN_ID);
        encoder = new CANcoder(61);
        this.modeSupplier = modeSupplier;
        


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
        pivotCfg.CurrentLimits.SupplyCurrentLimit = ArmConstants.CURRENT_LIMIT;
        pivotCfg.Feedback.SensorToMechanismRatio = 1.0; // adjust
        pivotCfg.Slot0.kP = 1.3; //1.31
        pivotCfg.MotionMagic.MotionMagicCruiseVelocity = ArmConstants.MAX_VELOCITY;
        pivotCfg.MotionMagic.MotionMagicAcceleration = ArmConstants.MAX_ACCELERATION;
        pivotCfg.MotionMagic.MotionMagicJerk = ArmConstants.MAX_JERK;
        pivotCfg.Feedback.FeedbackRemoteSensorID = encoder.getDeviceID();
        pivotCfg.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;  
        pivotCfg.ClosedLoopRamps.withDutyCycleClosedLoopRampPeriod(0.13);
        pivotCfg.ClosedLoopRamps.withVoltageClosedLoopRampPeriod(0.1);
        pivot.getConfigurator().apply(pivotCfg);

        motionMagic = new MotionMagicVoltage(0);
        double absolutePosition = encoder.getAbsolutePosition().getValueAsDouble(); // rotations [0,1)
        double adjustedPosition = absolutePosition - ArmConstants.PIVOT_HORIZONTAL_OFFSET_ROT;
        pivot.setPosition(adjustedPosition);
    }

    // Roller control
    public void intake() {
        roller.setControl(rollerDuty.withOutput(-0.4));
        rollerHolding = false;
    }

    public void eject(double speed) {
        roller.setControl(rollerDuty.withOutput(speed));
        rollerHolding = false;
    }
    public Command ejectPiece(double speed) {
        return Commands.run(
            () -> eject(speed), 
            this       
        ).withTimeout(0.17).finallyDo(interrupted -> stopOpenLoop());
    }

    public void stopOpenLoop() {
        roller.setControl(rollerDuty.withOutput(0.0));
    }

    // public boolean hasPiece() {
    //     double mm = 100;
    //     if (laser.getMeasurement()!=null){
    //         mm = laser.getMeasurement().distance_mm;}
            
    //     return mm < pieceThresholdMM;
    // }
    
    public boolean hasCoral() {  
        LaserCan.Measurement measurement = laser.getMeasurement();

        if(measurement != null && canTrust) {
            return measurement.distance_mm <= coralThresholdMM;
        }

        return false;
    }

    public boolean hasAlgae() {  
        LaserCan.Measurement measurement = laser.getMeasurement();

        if(measurement != null && canTrust) {
            return measurement.distance_mm <= algaeThresholdMM;
        }

        return false;
    }

    public boolean hasPiece() {  
        if(!hasAlgae() && !hasCoral()) {
            return false;
        }

        return true;
    }

    public Command intakePiece() {
        return Commands.run(
            () -> intake(), 
            this
        ).until(() -> hasPiece())
        .andThen(Commands.run(() -> intake(), this).withTimeout(0.15))
         .finallyDo(interrupted -> {stopOpenLoop();
            captureHoldFromEncoder();});
    }

    public boolean algaePos() {
        return algaePos;
    }

    public void setAlgaePos(boolean pos) {
        algaePos = pos;
    }

    // public void setPieceThresholdMM(double mm) {
    //     pieceThresholdMM = mm;
    // }

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
        holdPivotRot = encoder.getPosition().getValueAsDouble();
        pivot.setControl(pivotPosReq.withPosition(holdPivotRot));
        pivotHolding = true; //breaks at -0.03 -> -0.48
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

    public Command moveToPositionWithSpeed(ArmPositions position, double speed) {
        return Commands.run(() -> setPositionWithSpeed(position,speed), this).until(() -> atTarget(position)).andThen(() -> startHoldPivot());
    }

    public Command moveWhileRolling(ArmPositions position) {
        return Commands.run(() -> setPosition(position), this).alongWith(new InstantCommand(() -> eject(0.5))).until(() -> atTarget(position)).andThen(() -> startHoldPivot());
    }

    public Command moveWhileIntake(ArmPositions position) {
        return Commands.run(() -> setPosition(position), this).until(() -> atTarget(position)).andThen(() -> intakePiece()).andThen(() -> captureHoldFromEncoder());
    }

    public boolean atTarget(ArmPositions position) {
        double current = encoder.getPosition().getValueAsDouble();
        double error = Math.abs(position.getPosition() - current);
        return error < 0.085;
    }

    public double pivotAngleRad() {
        double cancoderRotations = encoder.getPosition().getValueAsDouble();
        double armRotations = (cancoderRotations - ArmConstants.PIVOT_HORIZONTAL_OFFSET_ROT) / 4.0;
        return armRotations * 2.0 * Math.PI;
    }
    

    public void setPosition(ArmConstants.ArmPositions position/* , boolean algae*/) {
        // if(algae) {
        //     armPosition = position.getPosition();
        //     pivot.setControl(pivotPosReq.withPosition(armPosition).withVelocity());
        // }
        armPosition = position.getPosition();
        pivot.setControl(pivotPosReq.withPosition(armPosition));
    }

    public void setPositionWithSpeed(ArmConstants.ArmPositions position, double speed) {
        armPosition = position.getPosition();
        pivot.setControl(pivotPosReq.withPosition(armPosition).withVelocity(speed));
    }
    
    public Command changeScoreSide() {
        return Commands.runOnce(() -> setScoreSide());
    }

    public void setScoreSide() {
        if(sideChange) {
            sideChange = false;
        } else {
            sideChange = true;
        }
    }

    public boolean checkScoreSide() {
        return sideChange;
    }

    public double getPositionDouble() {
        return encoder.getPosition().getValueAsDouble();
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Has Gamepiece?", hasPiece());
        SmartDashboard.putBoolean("Has Algae?", hasAlgae());

        LaserCan.Measurement measurement = laser.getMeasurement();
        if(measurement != null && measurement.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT) {
            trust++;
        } else {
            trust = 0;
        }

        canTrust = trust >= trustStrictness;
        
        SmartDashboard.putBoolean("Can Trust Laser?", canTrust);
        SmartDashboard.putNumber("Valid Streak", trust);
        if(measurement != null && canTrust) {
            SmartDashboard.putNumber("LaserMM", laser.getMeasurement().distance_mm);
            SmartDashboard.putNumber("LaserStatus", measurement.status);
            SmartDashboard.putBoolean("Yippee?", true);
        } else if(measurement != null){
            SmartDashboard.putNumber("LaserMM", laser.getMeasurement().distance_mm);
            SmartDashboard.putNumber("LaserStatus", measurement.status);
            SmartDashboard.putBoolean("Yippee?", false);
            if(measurement.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT && measurement.distance_mm == 0.0) {
                System.out.println("Straight lie");
            }
        }
        SmartDashboard.putNumber("PivotPos", pivot.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("RollerPos", roller.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("Encoder Relative", encoder.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("Encoder Value", encoder.getAbsolutePosition().getValueAsDouble());
        SmartDashboard.putBoolean("Is encoder working", encoder.isConnected());
        SmartDashboard.putNumber("Hold Position", holdPivotRot);
        SmartDashboard.putBoolean("Is Scoring Side Switched?", sideChange);
        SmartDashboard.putBoolean("At Target?", atTarget(ArmConstants.ArmPositions.PICKUP));
        SmartDashboard.putBoolean("Current Mode", modeSupplier.getAsBoolean()); //green for algae
        SmartDashboard.putBoolean("Scoring Side Flipped?", checkScoreSide());
        
    }
}
