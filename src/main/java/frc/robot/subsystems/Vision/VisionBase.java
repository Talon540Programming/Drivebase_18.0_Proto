package frc.robot.subsystems.Vision;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Drive.CommandSwerveDrivetrain;
import frc.robot.subsystems.Vision.VisionIO.VisionIOInputs;

public class VisionBase extends SubsystemBase{

    private final VisionIO vision;
    private final VisionIOInputs limelightOne = new VisionIOInputs();
    private final VisionIOInputs limelightTwo = new VisionIOInputs();
    private final CommandSwerveDrivetrain drivetrain;


    public VisionBase(VisionIO vision, CommandSwerveDrivetrain drivetrain) {
        this.vision = vision;
        this.drivetrain = drivetrain;
    }

    @Override
    public void periodic() {
        
        vision.updateLimelightYaw(drivetrain);
        vision.updateVisionIOInputs(limelightOne, limelightTwo);

        // Process both cameras
        processVisionMeasurement(limelightOne);
        processVisionMeasurement(limelightTwo);

        Logger.recordOutput("Vision/EstimatedPoseOne", limelightOne.pose);
        Logger.recordOutput("Vision/EstimatedPoseTwo", limelightTwo.pose);
    }

    private void processVisionMeasurement(VisionIOInputs input) {
        if (input.seenTagCount <= 0 || !input.hasTarget) {
            return;
        }

        // Reject if pose is outside the field
        if (input.pose.getX() < 0 || input.pose.getX() > 17.55 ||
            input.pose.getY() < 0 || input.pose.getY() > 8.05) {
            Logger.recordOutput("Vision/Rejected/" + input.cameraName, "Out of field bounds");
            return;
        }

        // Reject if pose jumped too far from current estimate (possible glitch)
        double poseDelta = drivetrain.getPose().getTranslation()
            .getDistance(input.pose.getTranslation());

        // Allow big corrections when vision is confident
        boolean visionIsConfident = input.seenTagCount >= 1 && input.avgTagDistance < 3.0;

        if (poseDelta > 1.5 && !visionIsConfident) {
            Logger.recordOutput("Vision/Rejected/" + input.cameraName, "Large jump with low confidence");
            return;
        }

        // Reject during fast rotation (motion blur causes bad readings)
        if (Math.abs(drivetrain.getFieldVelocity().omegaRadiansPerSecond) > Math.toRadians(180)) {
            Logger.recordOutput("Vision/Rejected/" + input.cameraName, "Rotating too fast");
            return;
        }

        Matrix<N3, N1> stdDevs = calculateStdDevs(input);
        drivetrain.addVisionMeasurement(input.pose, input.limelightTimestamp, stdDevs);
        Logger.recordOutput("Vision/Accepted/" + input.cameraName, input.pose);
    }

    private Matrix<N3, N1> calculateStdDevs(VisionIOInputs input) {
    // Base standard deviations (in meters for x/y, radians for rotation)
    double xyStdDev = 2;  // Start somewhat untrusting
    double rotStdDev = 2;
    
    // More tags = more confidence
    if (input.seenTagCount >= 2) {
        xyStdDev = 1;
        rotStdDev = 1;
    }
    
    if (input.avgTagDistance > 4.0) {
        xyStdDev *= 2.0;
        rotStdDev *= 2.0;
    }
    
    return VecBuilder.fill(xyStdDev, xyStdDev, rotStdDev);
    }

    public VisionIOInputs getVisionIOInputsOne() {
        return limelightOne;
    }
    
    public VisionIOInputs getVisionIOInputsTwo() {
        return limelightTwo;
    }
    
    public boolean hasTarget() {
        return limelightOne.hasTarget || limelightTwo.hasTarget;
    }
    
    public double getTX() {
        // Return TX from whichever camera has a target, prioritizing camera one
        if (limelightOne.hasTarget) {
            return limelightOne.limelightTX;
        }
        return limelightTwo.limelightTX;
    }

    public boolean isRedAlliance() {
        // Both should have the same value, just pick one
        return limelightOne.isRedAlliance;
    }    
}
