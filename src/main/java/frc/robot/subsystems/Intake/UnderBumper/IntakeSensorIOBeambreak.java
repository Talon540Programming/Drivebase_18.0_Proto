package frc.robot.subsystems.Intake.UnderBumper;

import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants.IntakeConstants;

public class IntakeSensorIOBeambreak implements IntakeSensorIO {

    private final DigitalInput beambreak;

    public IntakeSensorIOBeambreak() {
        beambreak = new DigitalInput(IntakeConstants.kIntakeBeambreakChannel);
    }

    @Override
    public void updateInputs(IntakeSensorIOInputs inputs) {
        // Beambreak is typically active-low (returns false when beam is broken)
        // Invert so hasGamePiece = true when beam is broken
        inputs.hasGamePiece = !beambreak.get();
        inputs.distanceMeters = 0.0; // Beambreak doesn't provide distance
    }
}