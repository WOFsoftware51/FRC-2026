// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Autons;

import edu.wpi.first.math.geometry.Rotation2d;
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

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Test extends SequentialCommandGroup {
  /** Creates a new test. */
  public Test(    
    Swerve swerve, 
    RobotState robotState, 
    ShooterSubsystem shooter, 
    IntakePivotSubsystem intakePivot,
    IntakeSubsystem intake, 
    FeederSubsystem feeder, 
    SpindexerSubsystem spindexer, 
    HoodSubsystem hood, 
    Superstructure superstructure
  ) {

    addCommands(
      Commands.runOnce(() -> swerve.resetRotation(new Rotation2d())),
      Commands.waitSeconds(1),
      // new MoveToAngle(
      //   swerve, 
      //   robotState, 
      //   robotState.getPose2d(),
      //   () -> robotState.justinTurretAngle(),
      //   // () -> robotState.getRobotToAllianceHubDegrees(),
      //   // () -> robotState.getTurretToAllianceHubDegrees(),
      //   1
      // ),
      // superstructure.shootTreeMap()

      superstructure.test()
    );
  }
}
