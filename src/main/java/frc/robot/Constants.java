// Copyright (c) 2025-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public final class Constants {
  public static final RobotType robot = Robot.isReal() ? RobotType.ALPHABOT : RobotType.SIMBOT;
  public static final boolean tuningMode = false;

  public static final double loopPeriodSecs = 0.02;
  public static final double loopPeriodWatchdogSecs = 0.2;

  public static Alliance getAllianceColor() {
    Alliance color = Alliance.Blue;
    
    if(!DriverStation.getAlliance().isPresent()){
      color = Alliance.Blue;
    }
    else {
      color = DriverStation.getAlliance().get();
    }

    return color;

  }

  public static Mode getMode() {
    return switch (robot) {
      case COMPBOT, ALPHABOT -> RobotBase.isReal() ? Mode.REAL : Mode.REPLAY;
      case SIMBOT -> Mode.SIM;
    };
  }

  public enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  public enum RobotType {
    COMPBOT,
    ALPHABOT,
    SIMBOT
  }

  public static boolean disableHAL = false;

  public static void disableHAL() {
    disableHAL = true;
  }

  /** Checks whether the correct robot is selected when deploying. */
  public static class CheckDeploy {
    public static void main(String... args) {
      if (robot == RobotType.SIMBOT) {
        System.err.println("Cannot deploy, invalid robot selected: " + robot);
        System.exit(1);
      }
    }
  }

  /** Checks that the default robot is selected and tuning mode is disabled. */
  public static class CheckPullRequest {
    public static void main(String... args) {
      if (robot != RobotType.COMPBOT || tuningMode) {
        System.err.println("Do not merge, non-default constants are configured.");
        System.exit(1);
      }
    }
  }

  public static final String kCANIvoreName = "CANivore";

  public static final class PoseConstants {
    private static final Pose2d kHubTargetBlue = new Pose2d(4.620, 4.040, new Rotation2d());
    private static final Pose2d kHubTargetRed = new Pose2d(11.915, 4.040, new Rotation2d());

    public static Supplier<Pose2d> kCurrentAllianceHubTarget = 
      () -> getAllianceColor() == Alliance.Blue ? kHubTargetBlue : kHubTargetRed;


    private static final Pose2d kRightStationTargetRed = new Pose2d(14.811, 6.271, new Rotation2d());
    private static final Pose2d kRightStationTargetBlue = new Pose2d(1.881, 1.835, new Rotation2d());

    public static Supplier<Pose2d> kCurrentAllianceRightStationTarget = 
      () -> getAllianceColor() == Alliance.Blue ? kRightStationTargetBlue : kRightStationTargetRed;


  }
  
  public static final class AutoConstants {
    public static final double kPDriveController = 4.0;

    public static final double kPThetaController = 3.0;
  }

  public static final class DriveConstants {
    public static final double kMaxSpeedMetersPerSecond = 4/1;
    public static final double kMaxAccelerationMetersPerSecondPerSecond = 4*2;

    public static final double kMaxOmegaRadiansPerSecond = 3*1;
    public static final double kMaxOmegaRadiansPerSecondPerSecond = 6*2;
  }

  public static final class IntakeConstants {
    public static final int kMotorID = 40;
    public static final double kGearRatio = 1.0;

  }

  public static final class IntakePivotConstants {
    public static final int kMotorID = 41;
    public static final int kCANcoderID = 2;
    public static final double kGearRatio = 80.0;
    public static final double kCANCoderGearRatio = 2.0; //2x slower than givto

    public static final double kCANCoderOffset = 0.459473;

    public static final double kForwardLimit = 0.0;
    public static final double kReverseLimit = 0.0;


  }


  public static final class SpindexerConstants {
    public static final int kMotorFrontID = 42;
    public static final int kMotorBackID = 43;
    public static final double kGearRatio = 1.0;
  }
  
  public static final class FeederConstants {
    public static final int kMotorID = 44;
    public static final double kGearRatio = 1.0;
  }
  
  public static final class TurretConstants {
    public static final int kMotorID = 46;
    public static final double kGearRatio = 45.3333;//37.33; 

    // public static final double kForwardLimit = 135.0;
    // public static final double kReverseLimit = -135;
    public static final double kForwardLimit = 290.0 - 10;
    public static final double kReverseLimit = -90.0 + 10.0;
        
  }
  
  public static final class HoodConstants {
    public static final int kMotorID = 48;
    public static final int kCANCoderID = 3;
    public static final double kGearRatio = 320.0;
    public static final double kCANCoderGearRatio = 9.0;

    public static final double kCANCoderOffset = -0.329102;


    public static final double kForwardLimit = 21;
    public static final double kReverseLimit = 0;

  }

  public static final class ShooterConstants {
    public static final int kMotorLeftID = 50;
    public static final int kMotorRightID = 51;
    public static final double kGearRatio = 1.0;
  }

  public static final class HangerConstants {
    public static final int kMotorLeftID = 52;
    public static final int kMotorRightID = 53;
    public static final double kGearRatio = 381.0;
  }

  public static final class VisionConstants {
    public static final String kChassisLimelight = "limelight-chassis";
    public static final String kTurretLimelight = "limelight-turret";
  }


}
