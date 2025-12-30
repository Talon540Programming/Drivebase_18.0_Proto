package frc.robot.subsystems.Intake.OverBumper;


import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.OverBumperConstants;
import frc.robot.subsystems.Intake.OverBumper.OverBumperIO.OverBumperIOInputs;


public class OverBumperBase extends SubsystemBase {

    private final OverBumperIO io;
    private final OverBumperIOInputs inputs = new OverBumperIOInputs();

    public OverBumperBase(OverBumperIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
    }

    // ==================== Getters ====================

    /** Get the current deploy position in rotations */
    public double getDeployPosition() {
        return inputs.deployPositionRotations;
    }

    /** Get the current roller velocity in RPS */
    public double getRollerVelocity() {
        return inputs.rollerVelocityRPS;
    }

    /** Check if the 4-bar is at the deployed position (within tolerance) */
    public boolean isDeployed() {
        return Math.abs(inputs.deployPositionRotations - OverBumperConstants.kDeployedPosition) 
            < OverBumperConstants.kPositionTolerance;
    }

    /** Check if the 4-bar is at the retracted position (within tolerance) */
    public boolean isRetracted() {
        return Math.abs(inputs.deployPositionRotations - OverBumperConstants.kRetractedPosition) 
            < OverBumperConstants.kPositionTolerance;
    }

    // ==================== Command Factories ====================

    /** Command to deploy the 4-bar linkage to the deployed position */
    public Command deployCommand() {
        return runOnce(() -> io.setDeployPosition(OverBumperConstants.kDeployedPosition))
            .withName("OverBumper Deploy");
    }

    /** Command to retract the 4-bar linkage to the retracted position */
    public Command retractCommand() {
        return runOnce(() -> io.setDeployPosition(OverBumperConstants.kRetractedPosition))
            .withName("OverBumper Retract");
    }

    /** Command to set the deploy position to a specific position */
    public Command setDeployPositionCommand(double positionRotations) {
        return runOnce(() -> io.setDeployPosition(positionRotations))
            .withName("OverBumper SetPosition");
    }

    /** Command to run the roller at the default intake velocity */
    public Command runRollerCommand() {
        return run(() -> io.setRollerVelocity(OverBumperConstants.kRollerIntakeVelocityRPS))
            .finallyDo(() -> io.stopRoller())
            .withName("OverBumper RunRoller");
    }

    /** Command to run the roller in reverse (eject) */
    public Command reverseRollerCommand() {
        return run(() -> io.setRollerVelocity(-OverBumperConstants.kRollerIntakeVelocityRPS))
            .finallyDo(() -> io.stopRoller())
            .withName("OverBumper ReverseRoller");
    }

    /** Command to set the roller to a specific velocity */
    public Command setRollerVelocityCommand(double velocityRPS) {
        return run(() -> io.setRollerVelocity(velocityRPS))
            .finallyDo(() -> io.stopRoller())
            .withName("OverBumper SetRollerVelocity");
    }

    /** Command to run the roller at a specific duty cycle */
    public Command setRollerDutyCycleCommand(double dutyCycle) {
        return run(() -> io.setRollerDutyCycle(dutyCycle))
            .finallyDo(() -> io.stopRoller())
            .withName("OverBumper SetRollerDutyCycle");
    }

    /** Command to stop the roller */
    public Command stopRollerCommand() {
        return runOnce(() -> io.stopRoller())
            .withName("OverBumper StopRoller");
    }

    /** Command to stop the deploy motor */
    public Command stopDeployCommand() {
        return runOnce(() -> io.stopDeploy())
            .withName("OverBumper StopDeploy");
    }

    /** Command to stop all motors */
    public Command stopAllCommand() {
        return runOnce(() -> io.stopAll())
            .withName("OverBumper StopAll");
    }

    /** 
     * Command to deploy and run the intake roller.
     * Deploys the 4-bar, then runs the roller until cancelled.
     */
    public Command deployAndIntakeCommand() {
        return deployCommand()
            .andThen(runRollerCommand())
            .withName("OverBumper DeployAndIntake");
    }

    /**
     * Command to retract and stop the intake roller.
     */
    public Command retractAndStopCommand() {
        return stopRollerCommand()
            .andThen(retractCommand())
            .withName("OverBumper RetractAndStop");
    }

    /**
     * Command to zero/home the deploy mechanism.
     * Should be run with the 4-bar against the retracted hard stop.
     */
    public Command zeroDeployCommand() {
        return runOnce(() -> io.resetDeployPosition(OverBumperConstants.kRetractedPosition))
            .withName("OverBumper ZeroDeploy");
    }
}