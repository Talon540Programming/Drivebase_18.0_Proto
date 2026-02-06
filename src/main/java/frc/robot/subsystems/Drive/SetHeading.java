package frc.robot.subsystems.Drive;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Util.HeadingCalculator;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.Vision.VisionBase;

/**
 * Manages automatic heading control to face the hub while driving.
 */
public class SetHeading {

    private final VisionBase vision;
    
    private boolean faceHubEnabled = false;
    private boolean passingEnabled = false;
    private Translation2d virtualGoal = null;

    public SetHeading(VisionBase vision) {
        this.vision = vision;
    }

    // ==================== HUB FACING MODE ====================

    public void toggleFaceHub() {
        faceHubEnabled = !faceHubEnabled;
        if (faceHubEnabled) {
            passingEnabled = false;
            Logger.recordOutput("Passing/Enabled", passingEnabled);
        }
        Logger.recordOutput("FaceHub/Enabled", faceHubEnabled);
    }

    public void disableFaceHub() {
        faceHubEnabled = false;
        Logger.recordOutput("FaceHub/Enabled", faceHubEnabled);
    }

    public boolean isEnabled() {
        return faceHubEnabled;
    }

    /**
     * Updates the virtual goal for shoot-while-moving.
     */
    public void updateVirtualGoal(Pose2d currentPose, ChassisSpeeds fieldVelocity) {
        boolean isRed = vision.isRedAlliance();
        virtualGoal = HeadingCalculator.calculateVirtualGoal(currentPose, fieldVelocity, isRed);
        
        Logger.recordOutput("FaceHub/VirtualGoal", virtualGoal);
        Logger.recordOutput("FaceHub/DistanceToHub", HeadingCalculator.getDistanceToHub(currentPose, isRed));
    }

    public Translation2d getVirtualGoal() {
        return virtualGoal;
    }

    // ==================== PASSING MODE ====================
    
    public void togglePassing() {
        passingEnabled = !passingEnabled;
        if (passingEnabled) {
            faceHubEnabled = false;
            Logger.recordOutput("FaceHub/Enabled", faceHubEnabled);
        }
        Logger.recordOutput("Passing/Enabled", passingEnabled);
    }

    public void disablePassing() {
        passingEnabled = false;
        Logger.recordOutput("Passing/Enabled", passingEnabled);
    }

    public boolean isPassingEnabled() {
        return passingEnabled;
    }

    public void updatePassingHeading(Pose2d currentPose) {
        boolean isRed = vision.isRedAlliance();
        double passingHeadingRad = HeadingCalculator.calculatePassingHeading(isRed);
        Logger.recordOutput("Passing/TargetHeadingDeg", Math.toDegrees(passingHeadingRad));
    }

    // ==================== DRIVER OVERRIDE ====================

    public boolean isDriverRotating(double rotationInput) {
        return Math.abs(rotationInput) > OperatorConstants.deadband;
    }
}