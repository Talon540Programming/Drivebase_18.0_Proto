package frc.robot.subsystems.Intake.OverBumper;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

import frc.robot.Constants.OverBumperConstants;
import frc.robot.generated.TunerConstants;

public class OverBumperIOKraken implements OverBumperIO {

    // Motors
    private final TalonFX deployMotor;
    private final TalonFX rollerMotor;

    // Deploy motor status signals
    private final StatusSignal<Angle> deployPosition;
    private final StatusSignal<AngularVelocity> deployVelocity;
    private final StatusSignal<Voltage> deployAppliedVolts;
    private final StatusSignal<Current> deployCurrentAmps;
    private final StatusSignal<Temperature> deployTempCelsius;

    // Roller motor status signals
    private final StatusSignal<AngularVelocity> rollerVelocity;
    private final StatusSignal<Voltage> rollerAppliedVolts;
    private final StatusSignal<Current> rollerCurrentAmps;
    private final StatusSignal<Temperature> rollerTempCelsius;

    // Control requests
    private final MotionMagicVoltage deployPositionControl = new MotionMagicVoltage(0);
    private final VelocityVoltage rollerVelocityControl = new VelocityVoltage(0);
    private final DutyCycleOut rollerDutyCycleControl = new DutyCycleOut(0);

    public OverBumperIOKraken() {
        deployMotor = new TalonFX(OverBumperConstants.kDeployMotorID, TunerConstants.kCANBus);
        rollerMotor = new TalonFX(OverBumperConstants.kRollerMotorID, TunerConstants.kCANBus);

        // Configure deploy motor (4-bar linkage with Motion Magic)
        TalonFXConfiguration deployConfig = new TalonFXConfiguration();
        
        deployConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        deployConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive; // PLACEHOLDER - adjust for your robot

        // Current limits
        deployConfig.CurrentLimits.StatorCurrentLimit = 40.0; // PLACEHOLDER - tune for your mechanism
        deployConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        deployConfig.CurrentLimits.SupplyCurrentLimit = 30.0; // PLACEHOLDER - tune for your mechanism
        deployConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

        // Slot 0 PID gains for Motion Magic position control
        deployConfig.Slot0.kP = 50.0; // PLACEHOLDER - tune for your mechanism
        deployConfig.Slot0.kI = 0.0; // PLACEHOLDER
        deployConfig.Slot0.kD = 0.5; // PLACEHOLDER
        deployConfig.Slot0.kS = 0.1; // PLACEHOLDER - static friction compensation
        deployConfig.Slot0.kV = 0.12; // PLACEHOLDER - velocity feedforward
        deployConfig.Slot0.kA = 0.0; // PLACEHOLDER - acceleration feedforward
        deployConfig.Slot0.kG = 0.2; // PLACEHOLDER - gravity compensation
        deployConfig.Slot0.GravityType = GravityTypeValue.Arm_Cosine; // 4-bar acts like an arm

        // Motion Magic settings
        deployConfig.MotionMagic.MotionMagicCruiseVelocity = 40.0; // PLACEHOLDER - rotations per second
        deployConfig.MotionMagic.MotionMagicAcceleration = 80.0; // PLACEHOLDER - rotations per second squared
        deployConfig.MotionMagic.MotionMagicJerk = 800.0; // PLACEHOLDER - rotations per second cubed

        // Soft limits (optional - can be enabled when positions are known)
        deployConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = false; // PLACEHOLDER - enable when tuned
        deployConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = OverBumperConstants.kDeployedPosition; // PLACEHOLDER
        deployConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = false; // PLACEHOLDER - enable when tuned
        deployConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = OverBumperConstants.kRetractedPosition; // PLACEHOLDER

        deployMotor.getConfigurator().apply(deployConfig);

        // Configure roller motor (velocity control)
        TalonFXConfiguration rollerConfig = new TalonFXConfiguration();

        rollerConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        rollerConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive; // PLACEHOLDER - adjust for your robot

        // Current limits
        rollerConfig.CurrentLimits.StatorCurrentLimit = 60.0; // PLACEHOLDER - tune for your mechanism
        rollerConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        rollerConfig.CurrentLimits.SupplyCurrentLimit = 40.0; // PLACEHOLDER - tune for your mechanism
        rollerConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

        // Slot 0 PID gains for velocity control
        rollerConfig.Slot0.kP = 0.1; // PLACEHOLDER - tune for your mechanism
        rollerConfig.Slot0.kI = 0.0; // PLACEHOLDER
        rollerConfig.Slot0.kD = 0.0; // PLACEHOLDER
        rollerConfig.Slot0.kS = 0.1; // PLACEHOLDER - static friction compensation
        rollerConfig.Slot0.kV = 0.12; // PLACEHOLDER - velocity feedforward

        rollerMotor.getConfigurator().apply(rollerConfig);

        // Set up status signals - Deploy motor
        deployPosition = deployMotor.getPosition();
        deployVelocity = deployMotor.getVelocity();
        deployAppliedVolts = deployMotor.getMotorVoltage();
        deployCurrentAmps = deployMotor.getStatorCurrent();
        deployTempCelsius = deployMotor.getDeviceTemp();

        // Set up status signals - Roller motor
        rollerVelocity = rollerMotor.getVelocity();
        rollerAppliedVolts = rollerMotor.getMotorVoltage();
        rollerCurrentAmps = rollerMotor.getStatorCurrent();
        rollerTempCelsius = rollerMotor.getDeviceTemp();

        // Set update frequencies
        BaseStatusSignal.setUpdateFrequencyForAll(
            50.0,
            deployPosition,
            deployVelocity,
            deployAppliedVolts,
            deployCurrentAmps,
            deployTempCelsius,
            rollerVelocity,
            rollerAppliedVolts,
            rollerCurrentAmps,
            rollerTempCelsius
        );

        deployMotor.optimizeBusUtilization();
        rollerMotor.optimizeBusUtilization();
    }

    @Override
    public void updateInputs(OverBumperIOInputs inputs) {
        BaseStatusSignal.refreshAll(
            deployPosition,
            deployVelocity,
            deployAppliedVolts,
            deployCurrentAmps,
            deployTempCelsius,
            rollerVelocity,
            rollerAppliedVolts,
            rollerCurrentAmps,
            rollerTempCelsius
        );

        inputs.deployPositionRotations = deployPosition.getValueAsDouble();
        inputs.deployVelocityRPS = deployVelocity.getValueAsDouble();
        inputs.deployAppliedVolts = deployAppliedVolts.getValueAsDouble();
        inputs.deployCurrentAmps = deployCurrentAmps.getValueAsDouble();
        inputs.deployTempCelsius = deployTempCelsius.getValueAsDouble();

        inputs.rollerVelocityRPS = rollerVelocity.getValueAsDouble();
        inputs.rollerAppliedVolts = rollerAppliedVolts.getValueAsDouble();
        inputs.rollerCurrentAmps = rollerCurrentAmps.getValueAsDouble();
        inputs.rollerTempCelsius = rollerTempCelsius.getValueAsDouble();
    }

    @Override
    public void setDeployPosition(double positionRotations) {
        deployMotor.setControl(deployPositionControl.withPosition(positionRotations));
    }

    @Override
    public void setRollerVelocity(double velocityRPS) {
        rollerMotor.setControl(rollerVelocityControl.withVelocity(velocityRPS));
    }

    @Override
    public void setRollerDutyCycle(double dutyCycle) {
        rollerMotor.setControl(rollerDutyCycleControl.withOutput(dutyCycle));
    }

    @Override
    public void stopDeploy() {
        deployMotor.stopMotor();
    }

    @Override
    public void stopRoller() {
        rollerMotor.stopMotor();
    }

    @Override
    public void stopAll() {
        stopDeploy();
        stopRoller();
    }

    @Override
    public void resetDeployPosition(double positionRotations) {
        deployMotor.setPosition(positionRotations);
    }
}