package frc.robot.subsystems.Intake.UnderBumper;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.Intake.UnderBumper.IntakeIO.IntakeIOInputs;
import frc.robot.subsystems.Intake.UnderBumper.IntakeSensorIO.IntakeSensorIOInputs;

public class IntakeBase extends SubsystemBase {

    private final IntakeIO io;
    private final IntakeSensorIO sensorIO;
    private final IntakeIOInputs inputs = new IntakeIOInputs();
    private final IntakeSensorIOInputs sensorInputs = new IntakeSensorIOInputs();

    public IntakeBase(IntakeIO io, IntakeSensorIO sensorIO) {
        this.io = io;
        this.sensorIO = sensorIO;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        sensorIO.updateInputs(sensorInputs);
        
        Logger.recordOutput("Intake/VelocityRPS", inputs.velocityRPS);
        Logger.recordOutput("Intake/AppliedVolts", inputs.appliedVolts);
        Logger.recordOutput("Intake/CurrentAmps", inputs.currentAmps);
        Logger.recordOutput("Intake/TempCelsius", inputs.tempCelsius);
        Logger.recordOutput("Intake/HasGamePiece", sensorInputs.hasGamePiece);
        Logger.recordOutput("Intake/SensorDistanceMeters", sensorInputs.distanceMeters);
    }

    /**
     * Returns whether the intake currently has a game piece
     */
    public boolean hasGamePiece() {
        return sensorInputs.hasGamePiece;
    }

    /**
     * Gets the current velocity of the intake roller in RPS
     */
    public double getVelocityRPS() {
        return inputs.velocityRPS;
    }

    // ==================== Commands ====================

    /**
     * Runs the intake inward to collect game pieces
     */
    public Command runIntakeCommand() {
        return run(() -> io.setDutyCycle(IntakeConstants.kIntakeSpeed))
            .withName("Intake.RunIntake");
    }

    /**
     * Runs the intake in reverse to eject game pieces
     */
    public Command reverseIntakeCommand() {
        return run(() -> io.setDutyCycle(-IntakeConstants.kIntakeSpeed))
            .withName("Intake.ReverseIntake");
    }

    /**
     * Runs the intake until a game piece is detected, then stops
     */
    public Command intakeUntilDetectedCommand() {
        return run(() -> io.setDutyCycle(IntakeConstants.kIntakeSpeed))
            .until(this::hasGamePiece)
            .finallyDo(() -> io.stop())
            .withName("Intake.IntakeUntilDetected");
    }

    /**
     * Stops the intake motor
     */
    public Command stopCommand() {
        return runOnce(() -> io.stop())
            .withName("Intake.Stop");
    }

    /**
     * Sets a custom duty cycle for the intake
     * @param dutyCycle The duty cycle to apply (-1.0 to 1.0)
     */
    public Command setDutyCycleCommand(double dutyCycle) {
        return run(() -> io.setDutyCycle(dutyCycle))
            .withName("Intake.SetDutyCycle");
    }
}