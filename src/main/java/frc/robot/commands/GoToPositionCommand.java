// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotState;
import frc.robot.subsystems.Swerve;
import frc.robot.util.LoggedTunableNumber;

public class GoToPositionCommand extends Command {
  private Swerve swerve;
  private RobotState robotState;
  
  private double driveErrorAbs;
  private double thetaErrorAbs;
  private double ffMinRadius = 0.0, ffMaxRadius = 0.1;
  private Supplier<Pose2d> targetLocationSupplier;
  private Pose2d targetLocation;

  private LoggedTunableNumber kPDriveController = new LoggedTunableNumber("DriveToPose/kPDriveController", 0.0);
  private LoggedTunableNumber kDDriveController = new LoggedTunableNumber("DriveToPose/kDDriveController", 0.0);
  private LoggedTunableNumber kPThetaController = new LoggedTunableNumber("DriveToPose/kPThetaController", 0.0);
  private LoggedTunableNumber kDThetaController = new LoggedTunableNumber("DriveToPose/kDThetaController", 0.0);
  
  public static final Translation2d kTranslation2dZero = new Translation2d();
  public static final Rotation2d kRotation2dZero = new Rotation2d();
  
  private ProfiledPIDController driveController;
  private ProfiledPIDController thetaController = 
    new ProfiledPIDController(
      Constants.AutoConstants.kPThetaController, 
      0, 
      0,
      new TrapezoidProfile.Constraints(
        Constants.DriveConstants.kMaxOmegaRadiansPerSecond, 
        Constants.DriveConstants.kMaxOmegaRadiansPerSecondPerSecond
      ),
      0.02
    );



  public GoToPositionCommand(
      Swerve swerve, 
      RobotState robotState, 
      Supplier<Pose2d> targetLocation,
      double constraintFactor) {
    this.swerve = swerve;
    this.targetLocationSupplier = targetLocation;
    this.robotState = robotState;

    this.driveController =
      new ProfiledPIDController(
        Constants.AutoConstants.kPDriveController,
        0.0,
        0.0,
        new TrapezoidProfile.Constraints(
          Constants.DriveConstants.kMaxSpeedMetersPerSecond * constraintFactor, 
          Constants.DriveConstants.kMaxAccelerationMetersPerSecondPerSecond * constraintFactor
        ),
        0.02
      );

    addRequirements(swerve);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);
  }

  public GoToPositionCommand(
      Swerve swerve, 
      RobotState robotState, 
      Pose2d targetLocation,
      double constraintFactor) {
    this.swerve = swerve;
    this.targetLocationSupplier = () -> targetLocation;
    this.robotState = robotState;

    this.driveController =
      new ProfiledPIDController(
        Constants.AutoConstants.kPDriveController,
        0.0,
        0.0,
        new TrapezoidProfile.Constraints(
          Constants.DriveConstants.kMaxSpeedMetersPerSecond * constraintFactor, 
          Constants.DriveConstants.kMaxAccelerationMetersPerSecondPerSecond * constraintFactor
        ),
        0.02
      );

    addRequirements(swerve);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Pose2d currentPose = robotState.getPose2d();
    targetLocation = targetLocationSupplier.get();
    
    driveController.reset(
      currentPose.getTranslation().getDistance(targetLocation.getTranslation()),
      Math.min(
        0.0,
        -new Translation2d(
          robotState.getFieldRelativeChassisSpeeds().vxMetersPerSecond,
          robotState.getFieldRelativeChassisSpeeds().vyMetersPerSecond
        )
        .rotateBy(
          targetLocation.getTranslation().minus(
          robotState.getPose2d().getTranslation()).getAngle().unaryMinus()
        )
        .getX()
      )
    );
    thetaController.reset(
      currentPose.getRotation().getRadians(),
      robotState.getFieldRelativeChassisSpeeds().omegaRadiansPerSecond
    );

    thetaController.setTolerance(Units.degreesToRadians(0.5));
    driveController.setTolerance(0.04);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    driveController.setP(kPDriveController.get());
    driveController.setD(kDDriveController.get());
    thetaController.setP(kPThetaController.get());
    thetaController.setD(kDThetaController.get());

    Pose2d currentPose = robotState.getPose2d();
    
    double[] targetLocationArray = new double[] {
      targetLocation.getX(),
      targetLocation.getY(),
      targetLocation.getRotation().getDegrees()
    };

    Logger.recordOutput("DriveToPose/targetPose", targetLocationArray);

    double currentDistance =
      currentPose.getTranslation().getDistance(targetLocation.getTranslation());

    double ffScaler =
      MathUtil.clamp(
        (currentDistance - ffMinRadius) / (ffMaxRadius - ffMinRadius), 0.0, 1.0);

    driveErrorAbs = currentDistance;
    Logger.recordOutput("DriveToPose/ffScaler", ffScaler);
    Logger.recordOutput("DriveToPose/driveErrorAbs", driveErrorAbs);
    double driveVelocityScalar =
        driveController.getSetpoint().velocity * ffScaler
        + driveController.calculate(driveErrorAbs, 0.0);
    if (currentDistance < driveController.getPositionTolerance()) driveVelocityScalar = 0.0;

    // Calculate theta speed
    double thetaVelocity =
      thetaController.getSetpoint().velocity * ffScaler
      + thetaController.calculate(
          currentPose.getRotation().getRadians(),
          targetLocation.getRotation().getRadians()
        );
    thetaErrorAbs = 
      Math.abs(currentPose.getRotation().minus(targetLocation.getRotation()).getRadians());
    if (thetaErrorAbs < thetaController.getPositionTolerance()) thetaVelocity = 0.0;


    // Command speeds
    var driveVelocity =
      new Pose2d(
        kTranslation2dZero,
        currentPose.getTranslation().minus(targetLocation.getTranslation()).getAngle()
        )
        .transformBy(
          new Transform2d(
          new Translation2d(driveVelocityScalar, 0.0), kRotation2dZero)
        )
        .getTranslation();

        swerve.setControl(
          new SwerveRequest.ApplyRobotSpeeds()
            .withSpeeds(
              ChassisSpeeds.fromFieldRelativeSpeeds(
                driveVelocity.getX(),
                driveVelocity.getY(),
                thetaVelocity,
                currentPose.getRotation()
              )
            ).withDriveRequestType(DriveRequestType.OpenLoopVoltage)
        );

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerve.setControl(new SwerveRequest.ApplyRobotSpeeds().withDriveRequestType(DriveRequestType.OpenLoopVoltage));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return targetLocation.equals(null)
      || (driveController.atGoal() && thetaController.atGoal());
  }
}
