// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Autons;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

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
public class Right_StopAtMiddle extends SequentialCommandGroup {

  public Right_StopAtMiddle(      
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
  ) 
  {
    try {
      addCommands(
        AutoBuilder.resetOdom(Paths.RightTrench_Center.getStartingHolonomicPose().get()), 
        Commands.parallel(
          AutoBuilder.followPath(Paths.emptyRightTrench),
          shooter.runRPMCommand(3000).withTimeout(0.05), 
          Commands.run(() -> turret.turretCameraAimToHub(), turret).withTimeout(0.02)
        ),
        Commands.race(
          AutoBuilder.followPath(Paths.RightTrench_Center), //go to center
          Commands.sequence(
            Commands.waitSeconds(0.25),
            intakePivot.goDown(), 
            Commands.waitSeconds(5)
          ),
          intake.runVolts(10.8)
        ), 
        AutoBuilder.followPath(Paths.RightPickUp_RightTrenchMoreCenter6), //go to shoot position
        Commands.race( //shoot
          Commands.sequence(
            Commands.waitSeconds(1.0),
            superstructure.shoot()
          ),
          Commands.run(() -> turret.turretCameraAimToHub(), turret),
          Commands.sequence(
            Commands.waitSeconds(2.5), 
            intakePivot.bounce().alongWith(intake.runVolts(6))
          ),
          Commands.waitSeconds(8)
        ), 
        Commands.parallel(
          Commands.race(
            AutoBuilder.followPath(Paths.RightTrenchMoreCenter_Center2), //go to center
            hood.runToPositionCommand(0)
          ),
          Commands.sequence(
            Commands.waitSeconds(0.5),
            intakePivot.goDown()
          )
        ),
        Commands.race(
          AutoBuilder.followPath(Paths.RightCenter_Pickup2), //go to center
          Commands.sequence(
            intakePivot.goDown(), 
            Commands.waitSeconds(5)
          ),
          intake.runVolts(10.8)
        )


      );

    }
    catch(Exception e) {
      e.printStackTrace();
    }
  }
}
