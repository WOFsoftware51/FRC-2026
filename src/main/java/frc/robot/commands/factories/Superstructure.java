package frc.robot.commands.factories;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.RobotState;
import frc.robot.commands.MoveToAngle;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.feeder.FeederSubsystem;
import frc.robot.subsystems.hood.HoodSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.intakePivot.IntakePivotSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.spindexer.SpindexerSubsystem;
import frc.robot.subsystems.turret.TurretSubsystem;
import frc.robot.subsystems.vision.VisionChassisSubsystem;
import frc.robot.subsystems.vision.VisionTurretSubsystem;

public class Superstructure {
    Swerve swerve;
    IntakeSubsystem intake;
    IntakePivotSubsystem intakePivot; 
    SpindexerSubsystem spindexer; 
    FeederSubsystem feeder; 
    TurretSubsystem turret;
    ShooterSubsystem shooter;
    HoodSubsystem hood;

    public Superstructure(
        Swerve swerve,
        IntakeSubsystem intake, 
        IntakePivotSubsystem intakePivot, 
        SpindexerSubsystem spindexer, 
        FeederSubsystem feeder, 
        TurretSubsystem turret, 
        ShooterSubsystem shooter, 
        HoodSubsystem hood
    ) {
        this.swerve = swerve;
        this.intake = intake;
        this.intakePivot = intakePivot;
        this.spindexer = spindexer;
        this.feeder = feeder; 
        this.turret = turret; 
        this.shooter = shooter;
        this.hood = hood;
    }

    public Command shoot() {
        return Commands.parallel(
            shooter.treeMapRPMCommand(), 
            Commands.sequence(
                Commands.waitUntil(() -> shooter.atRPM), 
                Commands.parallel(
                    feeder.run(), 
                    spindexer.run()
                )
            )
        );
        
    }

    public Command test() {
        var yes = 
            new MoveToAngle(
                swerve, 
                RobotState.getInstance(), 
                RobotState.getInstance().getPose2d(),
                () -> RobotState.getInstance().justinTurretAngle(),
                // () -> robotState.getRobotToAllianceHubDegrees(),
                // () -> robotState.getTurretToAllianceHubDegrees(),
                1
            ).withTimeout(2);

        Logger.recordOutput("yestesttest", yes.isFinished());
        
        return Commands.sequence(
            yes.alongWith(
                Commands.run(() -> Logger.recordOutput("yestesttest", yes.isFinished()))
            ), 
            shoot().withTimeout(3)
        );
    }

}
