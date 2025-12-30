package frc.robot.subsystems.Intake.UnderBumper;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {

    @AutoLog
    public static class IntakeIOInputs {
        public double velocityRPS = 0.0;
        public double appliedVolts = 0.0;
        public double currentAmps = 0.0;
        public double tempCelsius = 0.0;
        public boolean hasGamePiece = false;
    }

    /** Updates the set of loggable inputs */
    default void updateInputs(IntakeIOInputs inputs) {}

    /** Run the intake at a specified duty cycle (-1.0 to 1.0) */
    default void setDutyCycle(double dutyCycle) {}

    /** Stop the intake motor */
    default void stop() {}
}