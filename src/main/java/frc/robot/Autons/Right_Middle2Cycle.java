// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Autons;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
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
public class Right_Middle2Cycle extends SequentialCommandGroup {
  /** Creates a new Right_Middle2Cycle. */
  public Right_Middle2Cycle(
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
    PathPlannerPath RightTrench_Center;
    // PathPlannerPath RightCenter_Pickup;
    PathPlannerPath RightPickUp_RightTrench;
    PathPlannerPath RightTrench_Center2;
    PathPlannerPath RightCenter_Pickup2;
    PathPlannerPath RightPickUp_RightTrench2;
    PathPlannerPath emptyRightTrench;

    try {
      RightTrench_Center = PathPlannerPath.fromPathFile("RightTrench_Center");
      // RightCenter_Pickup = PathPlannerPath.fromPathFile("RightCenter_Pickup");
      RightPickUp_RightTrench = PathPlannerPath.fromPathFile("RightPickUp_RightTrench");
      RightTrench_Center2 = PathPlannerPath.fromPathFile("RightTrench_Center2");
      RightCenter_Pickup2 = PathPlannerPath.fromPathFile("RightCenter_Pickup2");
      RightPickUp_RightTrench2 = PathPlannerPath.fromPathFile("RightPickUp_RightTrench2");
      emptyRightTrench = PathPlannerPath.fromPathFile("emptyRightTrench");

      addCommands(
        AutoBuilder.resetOdom(RightTrench_Center.getStartingHolonomicPose().get()),
        Commands.parallel(
          AutoBuilder.followPath(emptyRightTrench),
          shooter.runRPMCommand(3000).withTimeout(0.05)
        ),
        Commands.race(
          AutoBuilder.followPath(RightTrench_Center), //go to center
          Commands.sequence(
            Commands.waitSeconds(0.25),
            intakePivot.goDown(), 
            intake.runVolts(10.8)
          )
        ),
        // AutoBuilder.followPath(RightCenter_Pickup).raceWith(intake.runVolts(10.8)), //pickup and intake
        AutoBuilder.followPath(RightPickUp_RightTrench), //go to shoot position
        Commands.race( //shoot
          superstructure.shoot(),
          Commands.run(() -> turret.turretCameraAimToHub()),
          Commands.sequence(
            Commands.waitSeconds(2.5), 
            intakePivot.bounce().alongWith(intake.runVolts(6))
          ),
          Commands.waitSeconds(5)
        ), 


        Commands.parallel(
          Commands.race(
            AutoBuilder.followPath(RightTrench_Center2), //go to center
            hood.runToPositionCommand(0)
          ),
          Commands.sequence(
            Commands.waitSeconds(0.5),
            intakePivot.goDown()
          )
        ),
        AutoBuilder.followPath(RightCenter_Pickup2).raceWith(intake.runVolts(10.56)), //pickup and intake again
        AutoBuilder.followPath(RightPickUp_RightTrench2), //go to shoot position again
        Commands.race( //shoot again
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
    catch(Exception e) {
      e.printStackTrace();
    }  

  }
}
