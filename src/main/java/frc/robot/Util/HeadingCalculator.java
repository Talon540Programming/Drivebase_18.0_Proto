package frc.robot.Util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants.FieldPoses;
import frc.robot.Constants.HeadingConstants;

/**
 * Calculates heading for auto-aim and shoot-while-moving.
 */
public class HeadingCalculator {

    /**
     * Get the shooter's position in field coordinates.
     * Accounts for shooter offset from robot center.
     */
    public static Translation2d getShooterPosition(Pose2d robotPose) {
        double offsetXMeters = Units.inchesToMeters(HeadingConstants.shooterOffsetXInches.get());
        double offsetYMeters = Units.inchesToMeters(HeadingConstants.shooterOffsetYInches.get());
        
        Transform2d shooterOffset = new Transform2d(
            new Translation2d(offsetXMeters, offsetYMeters),
            new Rotation2d()
        );
        
        Pose2d shooterPose = robotPose.transformBy(shooterOffset);
        return shooterPose.getTranslation();
    }

    /**
     * Calculate the heading to point the shooter at a target.
     */
    public static double calculateAimingHeading(Pose2d robotPose, Translation2d target) {
        Translation2d shooterPos = getShooterPosition(robotPose);
        
        double dx = target.getX() - shooterPos.getX();
        double dy = target.getY() - shooterPos.getY();
        
        return Math.atan2(dy, dx);
    }

    /**
     * Calculate distance from shooter to hub.
     */
    public static double getDistanceToHub(Pose2d robotPose, boolean isRedAlliance) {
        Pose2d hubPose = isRedAlliance ? FieldPoses.redHub : FieldPoses.blueHub;
        Translation2d shooterPos = getShooterPosition(robotPose);
        
        double dx = hubPose.getX() - shooterPos.getX();
        double dy = hubPose.getY() - shooterPos.getY();
        
        return Math.sqrt(dx * dx + dy * dy);
    }

    /**
     * Calculate virtual goal position for shoot-while-moving.
     * Uses a simplified fixed time-of-flight approach.
     */
    public static Translation2d calculateVirtualGoal(
            Pose2d robotPose, 
            ChassisSpeeds robotVelocity,
            boolean isRedAlliance) {
        
        Pose2d hubPose = isRedAlliance ? FieldPoses.redHub : FieldPoses.blueHub;
        Translation2d actualGoal = hubPose.getTranslation();
        
        // Use configurable time-of-flight
        double shotTime = HeadingConstants.timeOfFlightSeconds.get();
        
        // Calculate virtual goal by offsetting for robot velocity
        double virtualGoalX = actualGoal.getX() - shotTime * robotVelocity.vxMetersPerSecond * HeadingConstants.compensationFactorX.get();
        double virtualGoalY = actualGoal.getY() - shotTime * robotVelocity.vyMetersPerSecond * HeadingConstants.compensationFactorY.get();
        
        return new Translation2d(virtualGoalX, virtualGoalY);
    }

    /**
     * Calculate heading for passing (toward alliance wall).
     */
    public static double calculatePassingHeading(boolean isRedAlliance) {
        return isRedAlliance ? Math.PI : 0.0;
    }
}