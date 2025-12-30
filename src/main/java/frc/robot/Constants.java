// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ArrayList;
import java.util.List;

import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

import edu.wpi.first.math.util.Units;
/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final double deadband = 0.1;
  }

  public static final String kLimelightOne = "limelight-one";
  public static final String kLimelightTwo = "limelight-two";

  public static enum Reef {
    left,
    right,
    forward;
  }


  public static final class PathPlannerConstants {

    public static final PathConstraints testingConstraints = new PathConstraints(
        Units.feetToMeters(1.5), 2.0,             
        Units.degreesToRadians(50), Units.degreesToRadians(300));

    public static final PathConstraints slowConstraints = new PathConstraints(
        Units.feetToMeters(3.5), 4.0,             
        Units.degreesToRadians(100), Units.degreesToRadians(720));

    public static final PathConstraints defaultConstraints = new PathConstraints(
        Units.feetToMeters(8), 4.0,
        Units.degreesToRadians(200), Units.degreesToRadians(720));

    public static final PathConstraints fastConstraints = new PathConstraints(
      Units.feetToMeters(14), 8.0,
        Units.degreesToRadians(600), Units.degreesToRadians(1000));
  }

  public static final class FieldPoses {

    public static final double[] fieldSize = {17.55, 8.05};
    // Wall thickness is 0.051
    public static final Pose2d blueCenterOfReef = new Pose2d(4.487, 4.025, new Rotation2d()); // blue
    public static final Pose2d redCenterOfReef = new Pose2d(13.065, 4.025, new Rotation2d()); // red

    public static final List<Pose2d> blueReefPoses = new ArrayList<Pose2d>(){{
      add(new Pose2d(3.301, 4.026, new Rotation2d(Units.degreesToRadians(0))));      // Face 0 - Left
      add(new Pose2d(3.896, 2.999, new Rotation2d(Units.degreesToRadians(60.0))));   // Face 1
      add(new Pose2d(5.082, 2.999, new Rotation2d(Units.degreesToRadians(120.0))));  // Face 2
      add(new Pose2d(5.675, 4.026, new Rotation2d(Units.degreesToRadians(180.0))));  // Face 3 - Right
      add(new Pose2d(5.082, 5.053, new Rotation2d(Units.degreesToRadians(240.0))));  // Face 4
      add(new Pose2d(3.896, 5.053, new Rotation2d(Units.degreesToRadians(300.0))));  // Face 5
    }};

  public static final List<Pose2d> blueStationPoses = new ArrayList<Pose2d>(){{
    add(new Pose2d(1.38, 6.8, new Rotation2d(Units.degreesToRadians(-54)))); // Left Station
    add(new Pose2d(1.38, 1.19, new Rotation2d(Units.degreesToRadians(54)))); // Red Station
  }};

    
  public static final List<Pose2d> redReefPoses = new ArrayList<Pose2d>(){{
      add(new Pose2d(11.877, 4.026, new Rotation2d(Units.degreesToRadians(0))));     // Face 0
      add(new Pose2d(12.472, 2.999, new Rotation2d(Units.degreesToRadians(60))));  // Face 1
      add(new Pose2d(13.658, 2.999, new Rotation2d(Units.degreesToRadians(120.0)))); // Face 2
      add(new Pose2d(14.251, 4.026, new Rotation2d(Units.degreesToRadians(180.0)))); // Face 3
      add(new Pose2d(13.658, 5.053, new Rotation2d(Units.degreesToRadians(240.0)))); // Face 4
      add(new Pose2d(12.472, 5.053, new Rotation2d(Units.degreesToRadians(300.0)))); // Face 5
  }};

  public static final List<Pose2d> redStationPoses = new ArrayList<Pose2d>(){{
    add(new Pose2d(16.34, 1.19, new Rotation2d(Units.degreesToRadians(126)))); // Left Station
    add(new Pose2d(16.34, 6.8, new Rotation2d(Units.degreesToRadians(-126)))); // Red Station
  }};

  
    
    public static LoggedNetworkNumber reefLateralOffset = 
        new LoggedNetworkNumber("ReefAlign/LateralOffset", 0.2);
    public static LoggedNetworkNumber reefDistanceOffset = 
        new LoggedNetworkNumber("ReefAlign/ReefDistanceOffset", -0.05);
        public static LoggedNetworkNumber stationLateralOffset = 
        new LoggedNetworkNumber("ReefAlign/LateralOffset", 0.2);
    public static LoggedNetworkNumber stationDistanceOffset = 
        new LoggedNetworkNumber("ReefAlign/ReefDistanceOffset", 0.0);
        
    public static final double bumperWidth = -0.1;

  }

  // This will be added to Constants.java

  public static final class IntakeConstants {
    // Motor IDs
    public static final int kIntakeMotorID = 20; // PLACEHOLDER - update with actual CAN ID

    // Sensor IDs
    public static final int kIntakeSensorID = 30; // PLACEHOLDER - CANrange CAN ID
    public static final int kIntakeBeambreakChannel = 0; // PLACEHOLDER - DIO channel for beambreak

    // Sensor thresholds
    public static final double kIntakeDetectionThresholdMeters = 0.1; // PLACEHOLDER - distance threshold for CANrange

    // Motor speeds (duty cycle)
    public static final double kIntakeSpeed = 0.8; // PLACEHOLDER - tune for your mechanism

    // Simulation constants
    public static final double kIntakeMOI = 0.001; // PLACEHOLDER - moment of inertia in kg*m^2
    public static final double kIntakeGearRatio = 1.0; // PLACEHOLDER - gear ratio (motor rotations per roller rotation)
  }

  // ==================== OVER BUMPER CONSTANTS ====================
// Add this inner class to Constants.java

public static final class OverBumperConstants {
  // Motor CAN IDs
  public static final int kDeployMotorID = 21; // PLACEHOLDER - set to actual CAN ID
  public static final int kRollerMotorID = 22; // PLACEHOLDER - set to actual CAN ID

  // Deploy positions (in motor rotations after gear ratio)
  public static final double kRetractedPosition = 0.0; // PLACEHOLDER - retracted/home position
  public static final double kDeployedPosition = 2.5; // PLACEHOLDER - fully deployed position
  public static final double kPositionTolerance = 0.05; // PLACEHOLDER - tolerance for position checks

  // Roller speeds
  public static final double kRollerIntakeVelocityRPS = 40.0; // PLACEHOLDER - rotations per second

  // Simulation constants
  public static final double kDeployGearRatio = 50.0; // PLACEHOLDER - deploy motor gear ratio
  public static final double kDeployMOI = 0.5; // PLACEHOLDER - moment of inertia kg*m^2
  public static final double kDeployArmLengthMeters = 0.3; // PLACEHOLDER - effective arm length
  
  public static final double kRollerGearRatio = 2.0; // PLACEHOLDER - roller gear ratio
  public static final double kRollerMOI = 0.001; // PLACEHOLDER - roller moment of inertia kg*m^2
}

}
