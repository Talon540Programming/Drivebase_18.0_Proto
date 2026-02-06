package frc.robot.subsystems.Drive;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveControlParameters;
import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants.HeadingConstants;
import frc.robot.Util.HeadingCalculator;

/**
 * Custom swerve request that smoothly tracks a target point on the field.
 * Calculates heading internally at 250Hz (synced with odometry) to eliminate
 * the 4ms/20ms timing mismatch that causes module jitter.
 */
public class SmoothFieldCentricFacingAngle implements SwerveRequest {

    private final SwerveRequest.FieldCentric m_fieldCentric = new SwerveRequest.FieldCentric();

    public final PIDController HeadingController = new PIDController(5.0, 0.0, 0.0);

    private Translation2d m_redTarget = new Translation2d();
    private Translation2d m_blueTarget = new Translation2d();
    private Translation2d virtualTarget = null;

    // Request parameters
    public double velocityX = 0.0;
    public double velocityY = 0.0;
    public double deadband = 0.0;
    public double rotationalDeadband = 0.0;
    public SwerveModule.DriveRequestType driveRequestType = SwerveModule.DriveRequestType.OpenLoopVoltage;
    public SwerveModule.SteerRequestType steerRequestType = SwerveModule.SteerRequestType.MotionMagicExpo;
    public ForwardPerspectiveValue forwardPerspective = ForwardPerspectiveValue.OperatorPerspective;

    public SmoothFieldCentricFacingAngle() {
        HeadingController.enableContinuousInput(-Math.PI, Math.PI);
    }

    @Override
    public StatusCode apply(SwerveControlParameters parameters, SwerveModule<?, ?, ?>... modulesToApply) {
        
        Pose2d currentPose = parameters.currentPose;
        
        // Use virtual target if set, otherwise use alliance-based static target
        Translation2d target;
        if (virtualTarget != null) {
            target = virtualTarget;
        } else {
            target = m_blueTarget;
            var alliance = DriverStation.getAlliance();
            if (alliance.isPresent() && alliance.get() == Alliance.Red) {
                target = m_redTarget;
            }
        }
        
        // Calculate target heading at 250Hz with tunable offset
        double targetRadians = HeadingCalculator.calculateAimingHeading(currentPose, target) 
            + HeadingConstants.headingOffset.get();

        // Calculate rotation rate from PID
        double toApplyOmega = HeadingController.calculate(
            currentPose.getRotation().getRadians(),
            targetRadians
        );

        Logger.recordOutput("SmoothHeading/TargetRadians", targetRadians);
        Logger.recordOutput("SmoothHeading/CurrentRadians", currentPose.getRotation().getRadians());
        Logger.recordOutput("SmoothHeading/TotalOmega", toApplyOmega);

        return m_fieldCentric
            .withVelocityX(velocityX)
            .withVelocityY(velocityY)
            .withRotationalRate(toApplyOmega)
            .withDeadband(deadband)
            .withRotationalDeadband(rotationalDeadband)
            .withDriveRequestType(driveRequestType)
            .withSteerRequestType(steerRequestType)
            .withForwardPerspective(forwardPerspective)
            .apply(parameters, modulesToApply);
    }

    // Fluent API methods
    public SmoothFieldCentricFacingAngle withVelocityX(double velocityX) {
        this.velocityX = velocityX;
        return this;
    }

    public SmoothFieldCentricFacingAngle withVelocityY(double velocityY) {
        this.velocityY = velocityY;
        return this;
    }

    public SmoothFieldCentricFacingAngle withRedTarget(Translation2d target) {
        this.m_redTarget = target;
        return this;
    }

    public SmoothFieldCentricFacingAngle withBlueTarget(Translation2d target) {
        this.m_blueTarget = target;
        return this;
    }

    public SmoothFieldCentricFacingAngle withDeadband(double deadband) {
        this.deadband = deadband;
        return this;
    }

    public SmoothFieldCentricFacingAngle withRotationalDeadband(double rotationalDeadband) {
        this.rotationalDeadband = rotationalDeadband;
        return this;
    }

    public SmoothFieldCentricFacingAngle withDriveRequestType(SwerveModule.DriveRequestType driveRequestType) {
        this.driveRequestType = driveRequestType;
        return this;
    }

    public SmoothFieldCentricFacingAngle withSteerRequestType(SwerveModule.SteerRequestType steerRequestType) {
        this.steerRequestType = steerRequestType;
        return this;
    }

    public SmoothFieldCentricFacingAngle withForwardPerspective(ForwardPerspectiveValue forwardPerspective) {
        this.forwardPerspective = forwardPerspective;
        return this;
    }

    public SmoothFieldCentricFacingAngle withVirtualTarget(Translation2d virtualTarget) {
        this.virtualTarget = virtualTarget;
        return this;
    }

    public void clearVirtualTarget() {
        this.virtualTarget = null;
    }
}