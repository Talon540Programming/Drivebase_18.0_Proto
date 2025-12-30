package frc.robot.subsystems.Intake.OverBumper;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Constants.OverBumperConstants;

public class OverBumperIOSim implements OverBumperIO {

    // Simulate 4-bar as single jointed arm (similar kinematics)
    private final SingleJointedArmSim deploySim;
    private final FlywheelSim rollerSim;

    // Simple PID for simulated Motion Magic behavior
    private final PIDController deployPID = new PIDController(
        10.0, // PLACEHOLDER - simulation gains
        0.0,
        0.0
    );

    private double deployAppliedVolts = 0.0;
    private double rollerAppliedVolts = 0.0;
    private double deployTargetPosition = 0.0;
    private boolean deployPositionMode = false;

    public OverBumperIOSim() {
        // Create 4-bar simulation as a single jointed arm
        // Parameters are placeholders - adjust for your mechanism
        deploySim = new SingleJointedArmSim(
            DCMotor.getKrakenX60(1),
            OverBumperConstants.kDeployGearRatio, // PLACEHOLDER - gear ratio
            OverBumperConstants.kDeployMOI, // PLACEHOLDER - moment of inertia kg*m^2
            OverBumperConstants.kDeployArmLengthMeters, // PLACEHOLDER - arm length meters
            Math.toRadians(-10), // PLACEHOLDER - min angle radians (retracted)
            Math.toRadians(100), // PLACEHOLDER - max angle radians (deployed)
            true, // Simulate gravity
            Math.toRadians(0) // Starting angle
        );

        // Create roller simulation
        rollerSim = new FlywheelSim(
            LinearSystemId.createFlywheelSystem(
                DCMotor.getKrakenX60(1),
                OverBumperConstants.kRollerMOI, // PLACEHOLDER - moment of inertia
                OverBumperConstants.kRollerGearRatio // PLACEHOLDER - gear ratio
            ),
            DCMotor.getKrakenX60(1)
        );

        deployPID.setTolerance(0.01); // Position tolerance in rotations
    }

    @Override
    public void updateInputs(OverBumperIOInputs inputs) {
        // Update simulations with 20ms timestep
        
        // For deploy motor - if in position mode, calculate voltage from PID
        if (deployPositionMode) {
            double currentPositionRotations = radiansToRotations(deploySim.getAngleRads());
            double pidOutput = deployPID.calculate(currentPositionRotations, deployTargetPosition);
            deployAppliedVolts = MathUtil.clamp(pidOutput, -12.0, 12.0);
        }
        
        deploySim.setInputVoltage(deployAppliedVolts);
        deploySim.update(0.02);

        rollerSim.setInputVoltage(rollerAppliedVolts);
        rollerSim.update(0.02);

        // Deploy motor inputs (convert from radians to rotations)
        inputs.deployPositionRotations = radiansToRotations(deploySim.getAngleRads());
        inputs.deployVelocityRPS = deploySim.getVelocityRadPerSec() / (2.0 * Math.PI);
        inputs.deployAppliedVolts = deployAppliedVolts;
        inputs.deployCurrentAmps = deploySim.getCurrentDrawAmps();
        inputs.deployTempCelsius = 25.0; // Simulated temperature

        // Roller motor inputs
        inputs.rollerVelocityRPS = rollerSim.getAngularVelocityRPM() / 60.0;
        inputs.rollerAppliedVolts = rollerAppliedVolts;
        inputs.rollerCurrentAmps = rollerSim.getCurrentDrawAmps();
        inputs.rollerTempCelsius = 25.0; // Simulated temperature
    }

    @Override
    public void setDeployPosition(double positionRotations) {
        deployTargetPosition = positionRotations;
        deployPositionMode = true;
    }

    @Override
    public void setRollerVelocity(double velocityRPS) {
        // Simple velocity feedforward (approximate)
        // Real robot uses closed-loop, sim uses open-loop approximation
        rollerAppliedVolts = velocityRPS * 0.12 * 12.0; // PLACEHOLDER - rough feedforward
        rollerAppliedVolts = MathUtil.clamp(rollerAppliedVolts, -12.0, 12.0);
    }

    @Override
    public void setRollerDutyCycle(double dutyCycle) {
        rollerAppliedVolts = dutyCycle * 12.0;
    }

    @Override
    public void stopDeploy() {
        deployAppliedVolts = 0.0;
        deployPositionMode = false;
    }

    @Override
    public void stopRoller() {
        rollerAppliedVolts = 0.0;
    }

    @Override
    public void stopAll() {
        stopDeploy();
        stopRoller();
    }

    @Override
    public void resetDeployPosition(double positionRotations) {
        // In simulation, we reset by setting the sim state
        // SingleJointedArmSim doesn't have a direct setState method,
        // so we approximate by adjusting our internal tracking
        deploySim.setState(rotationsToRadians(positionRotations), 0.0);
    }

    /** Convert rotations to radians */
    private double rotationsToRadians(double rotations) {
        return rotations * 2.0 * Math.PI;
    }

    /** Convert radians to rotations */
    private double radiansToRotations(double radians) {
        return radians / (2.0 * Math.PI);
    }
}