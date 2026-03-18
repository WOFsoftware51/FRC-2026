// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Autons;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotState;
import frc.robot.commands.factories.Superstructure;
import frc.robot.subsystems.IntakePivotSubsystem;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.feeder.FeederSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.pivot.PivotSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.spindexer.SpindexerSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Right_Center extends SequentialCommandGroup {
  /** Creates a new Right_Center. */
  public Right_Center(Swerve swerve, RobotState robotState, ShooterSubsystem shooter, IntakePivotSubsystem intakePivot, IntakeSubsystem intake, FeederSubsystem feeder, SpindexerSubsystem spindexer, PivotSubsystem pivot, Superstructure superstructure) {
    PathPlannerPath RightTrench_Center;
    PathPlannerPath RightCenter_Pickup;
    PathPlannerPath RightPickUp_RightTrench;

    try {
      RightTrench_Center = PathPlannerPath.fromPathFile("RightTrench_Center");
      RightCenter_Pickup = PathPlannerPath.fromPathFile("RightCenter_Pickup");
      RightPickUp_RightTrench = PathPlannerPath.fromPathFile("RightPickUp_RightTrench");

      addCommands(
        AutoBuilder.resetOdom(RightTrench_Center.getStartingHolonomicPose().get()), 
        superstructure.test(),
        AutoBuilder.followPath(RightTrench_Center), 
        intakePivot.runPivotTimeBasedCommand(), 
        AutoBuilder.followPath(RightCenter_Pickup).raceWith(intake.runVolts(10.8)), 
        AutoBuilder.followPath(RightPickUp_RightTrench), 
        superstructure.test()

      );

    }
    catch(Exception e) {
      e.printStackTrace();
    }
  }
}
