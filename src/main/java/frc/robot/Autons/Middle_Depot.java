// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Autons;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.PathPlannerPaths;
import frc.robot.RobotState;
import frc.robot.commands.factories.Superstructure;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.feeder.FeederSubsystem;
import frc.robot.subsystems.hood.HoodSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.intakePivot.IntakePivotSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.spindexer.SpindexerSubsystem;
import frc.robot.subsystems.turret.TurretSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Middle_Depot extends SequentialCommandGroup {
  /** Creates a new Middle_Depot. */
  public Middle_Depot(
    Swerve swerve, 
    RobotState robotState, 
    ShooterSubsystem shooter, 
    TurretSubsystem turret, 
    IntakePivotSubsystem intakePivot,
    IntakeSubsystem intake, 
    FeederSubsystem feeder, 
    SpindexerSubsystem spindexer, 
    HoodSubsystem hood, 
    Superstructure superstructure, 
    PathPlannerPaths Paths
  ) {
    addCommands(
        AutoBuilder.resetOdom(Paths.Middle_Depot.getStartingHolonomicPose().get()), 
        Commands.race(
          AutoBuilder.followPath(Paths.Middle_Depot).withTimeout(0.05),
          shooter.runRPMCommand(3000).withTimeout(0.05)
        ), 
        Commands.race(
          AutoBuilder.followPath(Paths.Middle_Depot), //go to depot
          Commands.sequence(
            intakePivot.goDown(), 
            Commands.waitSeconds(20)
          ),
          intake.runVolts(10.8)
        ), 
        Commands.race( //shoot
          superstructure.shoot(),
          Commands.run(() -> turret.turretCameraAimToHub()),
          Commands.sequence(
            Commands.waitSeconds(2.5), 
            intakePivot.bounce().alongWith(intake.runVolts(6))
          ),
          Commands.waitSeconds(10)
        )
    );
  }
}
