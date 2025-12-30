package frc.robot.subsystems.Intake.UnderBumper;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.hardware.CANrange;

import edu.wpi.first.units.measure.Distance;
import frc.robot.Constants.IntakeConstants;
import frc.robot.generated.TunerConstants;

public class IntakeSensorIOCANrange implements IntakeSensorIO {

    private final CANrange sensor;
    
    private final StatusSignal<Distance> distanceSignal;
    private final StatusSignal<Boolean> detectedSignal;

    public IntakeSensorIOCANrange() {
        sensor = new CANrange(IntakeConstants.kIntakeSensorID, TunerConstants.kCANBus.getName());

        CANrangeConfiguration config = new CANrangeConfiguration();
        
        // Configure proximity detection threshold (in meters)
        config.ProximityParams.MinSignalStrengthForValidMeasurement = 2500; // PLACEHOLDER - tune for your sensor
        config.ProximityParams.ProximityThreshold = IntakeConstants.kIntakeDetectionThresholdMeters; // PLACEHOLDER - meters

        sensor.getConfigurator().apply(config);

        distanceSignal = sensor.getDistance();
        detectedSignal = sensor.getIsDetected();

        BaseStatusSignal.setUpdateFrequencyForAll(
            50.0,
            distanceSignal,
            detectedSignal
        );

        sensor.optimizeBusUtilization();
    }

    @Override
    public void updateInputs(IntakeSensorIOInputs inputs) {
        BaseStatusSignal.refreshAll(distanceSignal, detectedSignal);
        
        inputs.distanceMeters = distanceSignal.getValueAsDouble();
        inputs.hasGamePiece = detectedSignal.getValue();
    }
}