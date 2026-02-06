// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.Util.LoggedTunableNumber;

public final class Constants {

  public static final boolean TUNING_MODE = true;

  private static RobotType robotType = RobotType.SIMBOT;

  public static final String kLimelightOne = "limelight-one";
  public static final String kLimelightTwo = "limelight-two";

  public static final int PDHCanId = 1;

  public enum Mode {
    REAL,
    SIM,
    REPLAY
  }

  public enum RobotType {
    SIMBOT,
    COMPBOT
  }

  public static RobotType getRobot() {
    return robotType;
  }

  public static Mode getMode() {
    return switch (robotType) {
      case COMPBOT -> RobotBase.isReal() ? Mode.REAL : Mode.REPLAY;
      case SIMBOT -> Mode.SIM;
    };
  }

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final double deadband = 0.1;
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
    public static final double fieldLengthMeters = 17.55;
    public static final double fieldWidthMeters = 8.05;
    
    // Hub positions for REBUILT game
    public static final Pose2d blueHub = new Pose2d(4.5, 4.0, new Rotation2d());
    public static final Pose2d redHub = new Pose2d(12.0, 4.0, new Rotation2d());
  }

  public static final class HeadingConstants {
    // Heading PID
    public static LoggedTunableNumber headingP = new LoggedTunableNumber("Heading/P", 20.0);
    public static LoggedTunableNumber headingI = new LoggedTunableNumber("Heading/I", 0.0);
    public static LoggedTunableNumber headingD = new LoggedTunableNumber("Heading/D", 0.5);
    
    // Tunable heading offset (radians)
    public static LoggedTunableNumber headingOffset = 
        new LoggedTunableNumber("Heading/Offset", 0.0);

    // Shooter offset from robot center (inches)
    public static final LoggedTunableNumber shooterOffsetXInches = 
        new LoggedTunableNumber("Heading/ShooterOffsetXInches", 0.0);
    public static final LoggedTunableNumber shooterOffsetYInches = 
        new LoggedTunableNumber("Heading/ShooterOffsetYInches", 0.0);

    // Shoot-while-moving compensation
    public static final LoggedTunableNumber timeOfFlightSeconds = 
        new LoggedTunableNumber("Heading/TimeOfFlightSeconds", 0.5);
    public static final LoggedTunableNumber compensationFactorX = 
        new LoggedTunableNumber("Heading/CompensationFactorX", .1);
    public static final LoggedTunableNumber compensationFactorY = 
        new LoggedTunableNumber("Heading/CompensationFactorY", .1);
  }
}