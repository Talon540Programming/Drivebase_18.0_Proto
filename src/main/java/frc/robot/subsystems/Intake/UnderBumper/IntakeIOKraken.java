package frc.robot.subsystems.Intake.UnderBumper;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

import frc.robot.Constants.IntakeConstants;
import frc.robot.generated.TunerConstants;

public class IntakeIOKraken implements IntakeIO {

    private final TalonFX intakeMotor;
    
    private final StatusSignal<AngularVelocity> velocityRPS;
    private final StatusSignal<Voltage> appliedVolts;
    private final StatusSignal<Current> currentAmps;
    private final StatusSignal<Temperature> tempCelsius;

    private final DutyCycleOut dutyCycleControl = new DutyCycleOut(0);

    public IntakeIOKraken() {
        intakeMotor = new TalonFX(IntakeConstants.kIntakeMotorID, TunerConstants.kCANBus);

        TalonFXConfiguration config = new TalonFXConfiguration();
        
        // Motor output configuration
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive; // PLACEHOLDER - adjust for your robot

        // Current limits
        config.CurrentLimits.StatorCurrentLimit = 60.0; // PLACEHOLDER - tune for your mechanism
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit = 40.0; // PLACEHOLDER - tune for your mechanism
        config.CurrentLimits.SupplyCurrentLimitEnable = true;

        intakeMotor.getConfigurator().apply(config);

        // Set up status signals
        velocityRPS = intakeMotor.getVelocity();
        appliedVolts = intakeMotor.getMotorVoltage();
        currentAmps = intakeMotor.getStatorCurrent();
        tempCelsius = intakeMotor.getDeviceTemp();

        // Set update frequencies
        BaseStatusSignal.setUpdateFrequencyForAll(
            50.0,
            velocityRPS,
            appliedVolts,
            currentAmps,
            tempCelsius
        );
        
        intakeMotor.optimizeBusUtilization();
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        BaseStatusSignal.refreshAll(
            velocityRPS,
            appliedVolts,
            currentAmps,
            tempCelsius
        );
        
        inputs.velocityRPS = velocityRPS.getValueAsDouble();
        inputs.appliedVolts = appliedVolts.getValueAsDouble();
        inputs.currentAmps = currentAmps.getValueAsDouble();
        inputs.tempCelsius = tempCelsius.getValueAsDouble();
        // hasGamePiece will be updated by the sensor IO, not here
    }

    @Override
    public void setDutyCycle(double dutyCycle) {
        intakeMotor.setControl(dutyCycleControl.withOutput(dutyCycle));
    }

    @Override
    public void stop() {
        intakeMotor.stopMotor();
    }
}