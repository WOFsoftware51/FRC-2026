// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Autons;

import static edu.wpi.first.units.Units.*;

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

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Left_Center extends SequentialCommandGroup {
  PathPlannerPath LeftTrench_Center;
  PathPlannerPath Center_Pickup;
  PathPlannerPath PickUp_LeftTrench;
  
  public Left_Center (
    Swerve swerve, 
    RobotState robotState, 
    ShooterSubsystem shooter, 
    IntakePivotSubsystem intakePivot,
    IntakeSubsystem intake, 
    FeederSubsystem feeder, 
    SpindexerSubsystem spindexer, 
    HoodSubsystem hood, 
    Superstructure superstructure
    ) 
    {
      
    try {
      LeftTrench_Center = PathPlannerPath.fromPathFile("LeftTrench_Center");
      Center_Pickup = PathPlannerPath.fromPathFile("Center_Pickup");
      PickUp_LeftTrench = PathPlannerPath.fromPathFile("PickUp_LeftTrench");

      addCommands(
        AutoBuilder.resetOdom(LeftTrench_Center.getStartingHolonomicPose().get()),
        AutoBuilder.followPath(LeftTrench_Center),
        intakePivot.runSetpoint(0).withTimeout(3),
        AutoBuilder.followPath(Center_Pickup).raceWith(intake.runVolts(10.8)),
        AutoBuilder.followPath(PickUp_LeftTrench),
        new MoveToAngle(swerve, robotState, swerve.getState().Pose, () -> robotState.justinTurretAngle(), 1).withTimeout(3),
        shooter.treeMapRPMCommand()
      );

    }
    catch(Exception e) {
      e.printStackTrace();
    }
    
  }
}
