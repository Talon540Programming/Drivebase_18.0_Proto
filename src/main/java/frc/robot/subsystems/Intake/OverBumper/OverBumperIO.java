package frc.robot.subsystems.Intake.OverBumper;

import org.littletonrobotics.junction.AutoLog;

public interface OverBumperIO {

    @AutoLog
    public static class OverBumperIOInputs {
        // Deploy motor (4-bar linkage position)
        public double deployPositionRotations = 0.0;
        public double deployVelocityRPS = 0.0;
        public double deployAppliedVolts = 0.0;
        public double deployCurrentAmps = 0.0;
        public double deployTempCelsius = 0.0;
        
        // Roller motor
        public double rollerVelocityRPS = 0.0;
        public double rollerAppliedVolts = 0.0;
        public double rollerCurrentAmps = 0.0;
        public double rollerTempCelsius = 0.0;
    }

    /** Updates the set of loggable inputs */
    default void updateInputs(OverBumperIOInputs inputs) {}

    /** Set the deploy position using Motion Magic (rotations) */
    default void setDeployPosition(double positionRotations) {}

    /** Set the roller velocity (rotations per second) */
    default void setRollerVelocity(double velocityRPS) {}

    /** Run the roller at a specified duty cycle (-1.0 to 1.0) */
    default void setRollerDutyCycle(double dutyCycle) {}

    /** Stop the deploy motor */
    default void stopDeploy() {}

    /** Stop the roller motor */
    default void stopRoller() {}

    /** Stop all motors */
    default void stopAll() {}

    /** Reset the deploy encoder position (used for zeroing/homing) */
    default void resetDeployPosition(double positionRotations) {}
}