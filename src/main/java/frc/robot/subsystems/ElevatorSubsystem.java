package frc.robot.subsystems;
import static edu.wpi.first.units.Units.Rotation;


import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.StrictFollower;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorArrangementValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.ElevatorConstants.ElevatorPosition;

public class ElevatorSubsystem extends SubsystemBase {
    private final static ElevatorSubsystem INSTANCE = new ElevatorSubsystem();

    private final TalonFXS leftMotor;
    private final TalonFXS rightMotor;
    private final MotionMagicVoltage motionMagic;
    private final StrictFollower follower;
    private ElevatorPosition elevatorPosition = ElevatorPosition.GROUND;


    private final DutyCycleOut dutyCycleOut = new DutyCycleOut(0);
    private final PositionDutyCycle positionDutyCycle = new PositionDutyCycle(0);

    private double holdPositionRot = 0.0;
    private boolean holding = false;

    public ElevatorSubsystem() {
        leftMotor = new TalonFXS(ElevatorConstants.FRONT_MOTOR_ID);
        rightMotor = new TalonFXS(ElevatorConstants.BACK_MOTOR_ID);

        TalonFXSConfiguration config = new TalonFXSConfiguration();
        config.Commutation.MotorArrangement = MotorArrangementValue.Minion_JST;
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.Slot0.kP = 0.3; // tune PID
        config.Slot0.kI = 0.0;
        config.Slot0.kD = 0.0;
        config.Slot0.kG = ElevatorConstants.kFF;
        config.MotionMagic.MotionMagicCruiseVelocity = ElevatorConstants.MAX_VELOCITY;
        config.MotionMagic.MotionMagicAcceleration = ElevatorConstants.MAX_ACCELERATION;
        config.MotionMagic.MotionMagicJerk = ElevatorConstants.MAX_JERK;
        config.ExternalFeedback.SensorToMechanismRatio = ElevatorConstants.SENSOR_TO_MECHANISM_RATIO;
        
        CurrentLimitsConfigs limitConfigs = new CurrentLimitsConfigs();
        limitConfigs.StatorCurrentLimit = 120;
        limitConfigs.StatorCurrentLimitEnable = true;
        limitConfigs.SupplyCurrentLimit = Constants.ElevatorConstants.CURRENT_LIMIT;
        limitConfigs.SupplyCurrentLimitEnable = true;
        config.withCurrentLimits(limitConfigs);
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        leftMotor.getConfigurator().apply(config);
        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        rightMotor.getConfigurator().apply(config);

        leftMotor.setNeutralMode(NeutralModeValue.Brake);
        rightMotor.setNeutralMode(NeutralModeValue.Brake);

        follower = new StrictFollower(leftMotor.getDeviceID());
        rightMotor.setControl(follower);

        motionMagic = new MotionMagicVoltage(0);
    }

    public static ElevatorSubsystem getInstance() {
        return INSTANCE;
    }

    public void setPower(double power) {
        leftMotor.setControl(dutyCycleOut.withOutput(power));
        holding = false; // break out of hold when manually moving
    }

    public void setPosition(ElevatorPosition position) {
        elevatorPosition = position;
        leftMotor.setControl(motionMagic.withPosition(position.getHeight()));
    }

    public ElevatorPosition getElevatorPosition() {
        return elevatorPosition;
    }

    public void resetEncoder() {
        leftMotor.setPosition(0);
        rightMotor.setPosition(0);
    }

    public Command resetEncoderCommand(){
        return Commands.runOnce(() -> resetEncoder());
    }

    public Command moveToPosition(ElevatorPosition position) {
        SmartDashboard.putNumber("ElevatorSetpos", position.getHeight());
        return Commands.runOnce(() -> setPosition(position), this);
    }

    public Angle getCurrentPosition() {
        return leftMotor.getPosition().getValue();
    }

    public void addToPosition(double addRotations) {
        Angle currentPos = leftMotor.getPosition().getValue();
        leftMotor.setControl(motionMagic.withPosition(currentPos.in(Rotation) + addRotations));
    }

    @Override
    public void periodic() {
        // Add any periodic checks or logging here
        double currentPos = leftMotor.getPosition().getValue().magnitude();
        SmartDashboard.putNumber("ElevatorPos", currentPos);
        double setPointPos = leftMotor.getClosedLoopReference().getValue();
        SmartDashboard.putNumber("ElevatorSetPoint", setPointPos);
    }


}
