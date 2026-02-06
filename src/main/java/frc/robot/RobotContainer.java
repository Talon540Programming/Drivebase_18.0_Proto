// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import frc.robot.Constants.FieldPoses;
import frc.robot.Constants.HeadingConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Util.Telemetry;
import frc.robot.subsystems.Drive.CommandSwerveDrivetrain;
import frc.robot.subsystems.Drive.SetHeading;
import frc.robot.subsystems.Drive.SmoothFieldCentricFacingAngle;
import frc.robot.subsystems.Vision.VisionBase;
import frc.robot.subsystems.Vision.VisionIOLimelight;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.auto.AutoBuilder;
import com.ctre.phoenix6.swerve.SwerveRequest;

import frc.robot.generated.TunerConstants;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;

public class RobotContainer {
    private final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    private final VisionIOLimelight visionIO = new VisionIOLimelight();
    private final VisionBase vision = new VisionBase(visionIO, drivetrain);
    private final SetHeading autoHeading = new SetHeading(vision);

    private final SendableChooser<Command> autoChooser;

    private final CommandXboxController m_driverController =
        new CommandXboxController(OperatorConstants.kDriverControllerPort);

    private final SlewRateLimiter xLimiter = new SlewRateLimiter(4);
    private final SlewRateLimiter yLimiter = new SlewRateLimiter(4);
    private final SlewRateLimiter rotLimiter = new SlewRateLimiter(4);

    private static final double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    private double MaxAngularRate = RotationsPerSecond.of(1.5).in(RadiansPerSecond);

    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
        .withDeadband(0)
        .withRotationalDeadband(MaxAngularRate * 0.1)
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
  
    private final SmoothFieldCentricFacingAngle headingDrive = new SmoothFieldCentricFacingAngle()
        .withRedTarget(FieldPoses.redHub.getTranslation())
        .withBlueTarget(FieldPoses.blueHub.getTranslation())
        .withDeadband(MaxSpeed * 0.1)
        .withRotationalDeadband(0.0);
  
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    public RobotContainer() {
        configureBindings();
        
        autoChooser = AutoBuilder.buildAutoChooser("default auto");
        SmartDashboard.putData("Auto Chooser", autoChooser);
    }

    private void configureBindings() {
        // Start button - seed field centric
        m_driverController.start().onTrue(Commands.runOnce(() -> {
            drivetrain.seedFieldCentric();
            vision.setGyroInitialized();
        }));

        // B button - brake
        m_driverController.b().whileTrue(drivetrain.applyRequest(() -> brake));

        // Right bumper - toggle hub facing mode
        m_driverController.rightBumper().onTrue(Commands.runOnce(() -> {
            if (autoHeading.isEnabled()) {
                autoHeading.disableFaceHub();
            } else {
                autoHeading.toggleFaceHub();
            }
        }));

        // Left bumper - toggle passing mode
        m_driverController.leftBumper().onTrue(Commands.runOnce(() -> {
            if (autoHeading.isPassingEnabled()) {
                autoHeading.disablePassing();
            } else {
                autoHeading.togglePassing();
            }
        }));

        // Configure heading PID
        headingDrive.HeadingController.enableContinuousInput(-Math.PI, Math.PI);
        headingDrive.HeadingController.setPID(
            HeadingConstants.headingP.get(), 
            HeadingConstants.headingI.get(), 
            HeadingConstants.headingD.get()
        );

        // Default drive command
        drivetrain.setDefaultCommand(
            drivetrain.run(() -> {
                // Get raw joystick inputs
                double rawX = -m_driverController.getLeftY();
                double rawY = -m_driverController.getLeftX();
                double rawRot = -m_driverController.getRightX();
                
                // Apply deadband BEFORE the limiter
                rawX = Math.abs(rawX) > OperatorConstants.deadband ? rawX : 0.0;
                rawY = Math.abs(rawY) > OperatorConstants.deadband ? rawY : 0.0;
                rawRot = Math.abs(rawRot) > OperatorConstants.deadband ? rawRot : 0.0;
                
                // Apply slew rate limiting
                double xSpeed = xLimiter.calculate(rawX);
                double ySpeed = yLimiter.calculate(rawY);
                double rotSpeed = rotLimiter.calculate(rawRot);
                
                // Check if driver is manually rotating
                boolean driverRotating = autoHeading.isDriverRotating(rotSpeed);

                // Determine which auto-heading mode to use
                boolean useHubHeading = autoHeading.isEnabled() && !driverRotating;
                boolean usePassingHeading = autoHeading.isPassingEnabled() && !driverRotating;

                // Update heading based on active mode
                if (useHubHeading) {
                    autoHeading.updateVirtualGoal(drivetrain.getPose(), drivetrain.getFieldVelocity());
                    headingDrive.withVirtualTarget(autoHeading.getVirtualGoal());
                } else if (usePassingHeading) {
                    autoHeading.updatePassingHeading(drivetrain.getPose());
                    // For passing, point toward alliance wall
                    boolean isRed = vision.isRedAlliance();
                    double targetX = isRed ? FieldPoses.fieldLengthMeters + 10.0 : -10.0;
                    headingDrive.withVirtualTarget(new Translation2d(targetX, drivetrain.getPose().getY()));
                } else {
                    headingDrive.clearVirtualTarget();
                }

                if (useHubHeading || usePassingHeading) {
                    drivetrain.setControl(
                        headingDrive
                            .withVelocityX(xSpeed * MaxSpeed)
                            .withVelocityY(ySpeed * MaxSpeed)
                    );
                } else {
                    drivetrain.setControl(
                        drive
                            .withVelocityX(xSpeed * MaxSpeed)
                            .withVelocityY(ySpeed * MaxSpeed)
                            .withRotationalRate(rotSpeed * MaxAngularRate)
                    );
                }
            })
        );

        // Idle while disabled
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public void finalGyroCheck() {
        if (!vision.isGyroInitialized()) {
            boolean success = vision.forceSetYawFromCameras(drivetrain);
            if (!success) {
                drivetrain.seedFieldCentric();
                vision.setGyroInitialized();
            }
        }
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}