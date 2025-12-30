package frc.robot.subsystems.Intake.UnderBumper;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class IntakeSensorIOSim implements IntakeSensorIO {

    public IntakeSensorIOSim() {
        // Initialize SmartDashboard toggle for simulation
        SmartDashboard.putBoolean("Sim/IntakeHasGamePiece", false);
    }

    @Override
    public void updateInputs(IntakeSensorIOInputs inputs) {
        // Allow manual control of game piece detection in simulation via SmartDashboard
        inputs.hasGamePiece = SmartDashboard.getBoolean("Sim/IntakeHasGamePiece", false);
        inputs.distanceMeters = inputs.hasGamePiece ? 0.05 : 0.5; // Simulated distance
    }
}