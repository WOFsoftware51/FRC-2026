// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Autons;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotState;
import frc.robot.commands.MoveToAngle;
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
public class Left_StopAtMiddle extends SequentialCommandGroup {
  /** Creates a new Left_StopAtMiddle. */
    PathPlannerPath LeftTrench_Center6;
    PathPlannerPath emptyLeftTrench6;
    PathPlannerPath LeftPickUp_LeftTrenchMoreCenter;
    PathPlannerPath LeftTrenchMoreCenter_LeftCenter2;
    PathPlannerPath LeftCenter_Pickup26;


  public Left_StopAtMiddle(
      Swerve swerve, 
      RobotState robotState, 
      ShooterSubsystem shooter, 
      TurretSubsystem turret, 
      IntakePivotSubsystem intakePivot,
      IntakeSubsystem intake, 
      FeederSubsystem feeder, 
      SpindexerSubsystem spindexer, 
      HoodSubsystem hood, 
      Superstructure superstructure
    ) 
    {
    try {
      LeftTrench_Center6 = PathPlannerPath.fromPathFile("LeftTrench_Center6");
      emptyLeftTrench6 = PathPlannerPath.fromPathFile("emptyLeftTrench6");
      LeftPickUp_LeftTrenchMoreCenter = PathPlannerPath.fromPathFile("LeftPickUp_LeftTrenchMoreCenter");
      LeftTrenchMoreCenter_LeftCenter2 = PathPlannerPath.fromPathFile("LeftTrenchMoreCenter_LeftCenter2");
      LeftCenter_Pickup26 = PathPlannerPath.fromPathFile("LeftCenter_Pickup26");

      addCommands(
        AutoBuilder.resetOdom(LeftTrench_Center6.getStartingHolonomicPose().get()), 
        Commands.parallel(
          AutoBuilder.followPath(emptyLeftTrench6),
          shooter.runRPMCommand(3000).withTimeout(0.05)
        ),
        Commands.race(
          AutoBuilder.followPath(LeftTrench_Center6), //go to center
          Commands.sequence(
            Commands.waitSeconds(0.25),
            intakePivot.goDown(), 
            Commands.waitSeconds(5)
          ),
          intake.runVolts(10.8)
        ), 
        AutoBuilder.followPath(LeftPickUp_LeftTrenchMoreCenter), //go to shoot position
        Commands.race( //shoot
          Commands.sequence(
            Commands.waitSeconds(1.0),
            superstructure.shoot()
          ),
          Commands.run(() -> turret.turretCameraAimToHub()),
          Commands.sequence(
            Commands.waitSeconds(2.5), 
            intakePivot.bounce().alongWith(intake.runVolts(6))
          ),
          Commands.waitSeconds(8)
        ), 
        Commands.parallel(
          Commands.race(
            AutoBuilder.followPath(LeftTrenchMoreCenter_LeftCenter2), //go to center
            hood.runToPositionCommand(0)
          ),
          Commands.sequence(
            Commands.waitSeconds(0.5),
            intakePivot.goDown()
          )
        ), 
        Commands.race(
          AutoBuilder.followPath(LeftCenter_Pickup26), //go to center
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
