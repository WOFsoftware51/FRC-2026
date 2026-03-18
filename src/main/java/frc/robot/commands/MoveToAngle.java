// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.Radians;

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
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotState;
import frc.robot.subsystems.Swerve;
import frc.robot.util.LoggedTunableNumber;

public class MoveToAngle extends Command {
  private Swerve swerve;
  private Pose2d currentPose;
  private Supplier<Angle> targetAngle;
  private RobotState robotState;
  
  private double thetaErrorAbs;
  private double ffMinRadius = 0.0,   ffMaxRadius = 0.01;
  private Angle targetLocation;

  private LoggedTunableNumber kPThetaController = new LoggedTunableNumber("MoveToAngle/kPThetaController", 0.0);
  private LoggedTunableNumber kDThetaController = new LoggedTunableNumber("MoveToAngle/kDThetaController", 0.0);
  
  public static final Translation2d kTranslation2dZero = new Translation2d();
  public static final Rotation2d kRotation2dZero = new Rotation2d();
  
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



  public MoveToAngle(
      Swerve swerve, 
      RobotState robotState,
      Pose2d currentPose, 
      Supplier<Angle> targetAngle,
      double constraintFactor) {
    this.swerve = swerve;
    this.targetAngle = targetAngle;
    this.robotState = robotState;


    addRequirements(swerve);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.currentPose = robotState.getPose2d();
    targetLocation = targetAngle.get();

    
    thetaController.reset(
      currentPose.getRotation().getRadians(),
      robotState.getFieldRelativeChassisSpeeds().omegaRadiansPerSecond
    );

    thetaController.setTolerance(Units.degreesToRadians(0.5));

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    thetaController.setP(kPThetaController.get());
    thetaController.setD(kDThetaController.get());

    Pose2d currentPose = robotState.getPose2d();
    
    double currentDistance =
      currentPose.getRotation().getRadians() - (targetLocation.in(Radians));

    Logger.recordOutput("MoveToAngle/error", currentDistance);
    
    double ffScaler =
      MathUtil.clamp(
        Math.abs((currentDistance - ffMinRadius) / (ffMaxRadius - ffMinRadius)), 0.0, 1.0);

    Logger.recordOutput("MoveToAngle/ffScaler", ffScaler);

    // Calculate theta speed
    double thetaVelocity =
      thetaController.getSetpoint().velocity * ffScaler
      + thetaController.calculate(
          currentPose.getRotation().getRadians(),
          targetLocation.in(Radians)
        );
    thetaErrorAbs = 
      Math.abs(currentPose.getRotation().minus(new Rotation2d(targetLocation)).getRadians());
    if (thetaErrorAbs < thetaController.getPositionTolerance()) thetaVelocity = 0.0;

        swerve.setControl(
          new SwerveRequest.ApplyRobotSpeeds()
            .withSpeeds(
              ChassisSpeeds.fromFieldRelativeSpeeds(
                0,
                0,
                thetaVelocity,
                currentPose.getRotation()
              )
            ).withDriveRequestType(DriveRequestType.Velocity)
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
    return targetLocation == null
      || (thetaController.atGoal());
  }
}
