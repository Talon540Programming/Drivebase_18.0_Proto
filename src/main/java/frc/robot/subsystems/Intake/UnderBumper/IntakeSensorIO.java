package frc.robot.subsystems.Intake.UnderBumper;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeSensorIO {

    @AutoLog
    public static class IntakeSensorIOInputs {
        public boolean hasGamePiece = false;
        public double distanceMeters = 0.0; // Only used for CANrange
    }

    /** Updates the set of loggable inputs */
    default void updateInputs(IntakeSensorIOInputs inputs) {}
}