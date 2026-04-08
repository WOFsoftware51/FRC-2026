// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Autons.Left_Center;
import frc.robot.Autons.Left_Middle2Cycle;
import frc.robot.Autons.Left_StopAtMiddle;
import frc.robot.Autons.Right_Center;
import frc.robot.Autons.Right_Middle2Cycle;
import frc.robot.Autons.Right_StopAtMiddle;
import frc.robot.Autons.Test;
import frc.robot.Autons.doNOTHING;
import frc.robot.commands.MoveToAngle;
import frc.robot.commands.TurretCameraPoseDefaultCommand;
import frc.robot.commands.factories.Superstructure;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.feeder.FeederIOHardware;
import frc.robot.subsystems.feeder.FeederIOSim;
import frc.robot.subsystems.feeder.FeederSubsystem;
import frc.robot.subsystems.hood.HoodIOHardware;
import frc.robot.subsystems.hood.HoodIOSim;
import frc.robot.subsystems.hood.HoodSubsystem;
import frc.robot.subsystems.intake.IntakeIOHardware;
import frc.robot.subsystems.intake.IntakeIOSim;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.intakePivot.IntakePivotIOHardware;
import frc.robot.subsystems.intakePivot.IntakePivotIOSim;
import frc.robot.subsystems.intakePivot.IntakePivotSubsystem;
import frc.robot.subsystems.shooter.ShooterIOHardware;
import frc.robot.subsystems.shooter.ShooterIOSim;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.spindexer.SpindexerIOHardware;
import frc.robot.subsystems.spindexer.SpindexerIOSim;
import frc.robot.subsystems.spindexer.SpindexerSubsystem;
import frc.robot.subsystems.turret.TurretIOHardware;
import frc.robot.subsystems.turret.TurretIOSim;
import frc.robot.subsystems.turret.TurretSubsystem;
import frc.robot.subsystems.turret.TurretSubsystem.Targets;
import frc.robot.subsystems.vision.VisionChassisSubsystem;
import frc.robot.subsystems.vision.VisionIOHardware;
import frc.robot.subsystems.vision.VisionIOSim;
import frc.robot.subsystems.vision.VisionTurretSubsystem;
import frc.robot.util.LoggedTunableNumber;

public class RobotContainer {
    public double Speedmodifier = 0.5;
    private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(1.2).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity: 0.8435211984 RPS


    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.FieldCentric poseTuning = new SwerveRequest.FieldCentric()
            .withDriveRequestType(DriveRequestType.Velocity); 
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController driver = new CommandXboxController(0);
    private final CommandXboxController operator = new CommandXboxController(1);
    
    private final CommandXboxController joystick = new CommandXboxController(3);
    private final CommandXboxController test = new CommandXboxController(5);

    private final SendableChooser<Integer> a_chooser = new SendableChooser<>();


    private final VisionTurretSubsystem limelightTurret;
    private final VisionChassisSubsystem limelightChassis;

    public final Swerve swerve;
    public final RobotState robotState = RobotState.getInstance();
    private final TurretSubsystem turret;
    private final ShooterSubsystem shooter;
    private final HoodSubsystem hood;
    private final FeederSubsystem feeder;
    private final SpindexerSubsystem spindexer;
    private final IntakeSubsystem intake;
    private final IntakePivotSubsystem intakePivot;
    private final Superstructure superstructure;

    private boolean operatorRumble = false;
    

    LoggedTunableNumber speedLeft = new LoggedTunableNumber("Pose/speedLeft", 5);
    LoggedTunableNumber speedRight = new LoggedTunableNumber("Pose/speedRight", 5);
    LoggedTunableNumber speedForward = new LoggedTunableNumber("Pose/speedLeft", 1);
    LoggedTunableNumber speedBackward = new LoggedTunableNumber("Pose/speedRight", 1);
    
    LoggedTunableNumber pivotAngle = new LoggedTunableNumber("Shooting/Pivot", 0.0);
    LoggedTunableNumber turretAngle = new LoggedTunableNumber("Shooting/Turret", 0.0);


    public RobotContainer() {
        this.intakePivot = new IntakePivotSubsystem(
            Robot.isReal() ? new IntakePivotIOHardware() : new IntakePivotIOSim()
        );
        this.limelightTurret = new VisionTurretSubsystem(
            //  new VisionIOHardware(Constants.VisionConstants.kTurretLimelight)
            Robot.isReal() ? new VisionIOHardware(Constants.VisionConstants.kTurretLimelight) : new VisionIOSim()
        );

        this.spindexer = new SpindexerSubsystem(
            Robot.isReal() ? new SpindexerIOHardware() : new SpindexerIOSim()
        );

        this.feeder = new FeederSubsystem(
            Robot.isReal() ? new FeederIOHardware() : new FeederIOSim()
        );

        this.hood = new HoodSubsystem(
            Robot.isReal() ? new HoodIOHardware() : new HoodIOSim(), 
            robotState
        );
        
        this.limelightChassis = new VisionChassisSubsystem(
            Robot.isReal() ? new VisionIOHardware(Constants.VisionConstants.kChassisLimelight) : new VisionIOSim()
        );

        this.turret = new TurretSubsystem(
            Robot.isReal() ? new TurretIOHardware() : new TurretIOSim(),
            limelightTurret, 
            robotState
        );

        this.swerve = TunerConstants.createDrivetrain(limelightTurret, limelightChassis);

        this.intake = new IntakeSubsystem(
            Robot.isReal() ? new IntakeIOHardware() : new IntakeIOSim(swerve.mapleSimSwerveDrivetrain.mapleSimDrive)
        );

        this.shooter = new ShooterSubsystem(
            Robot.isReal() ? new ShooterIOHardware() : new ShooterIOSim(this.swerve.mapleSimSwerveDrivetrain.mapleSimDrive)
        );
        
        this.superstructure = new Superstructure(swerve, intake, intakePivot, spindexer, feeder, turret, shooter, hood);

        configureBindings();
        printAutons();
    }

    private void configureBindings() {
        /*
        Swerve Controls
        */
            // Note that X is defined as forward according to WPILib convention,
            // and Y is defined as to the left according to WPILib convention.
            swerve.setDefaultCommand(

                // Drivetrain will execute this command periodically
                swerve.applyRequest(() ->
                    drive.withVelocityX(-driver.getLeftY() * MaxSpeed * Speedmodifier) // Drive forward with negative Y (forward)
                        .withVelocityY(-driver.getLeftX() * MaxSpeed * Speedmodifier) // Drive left with negative X (left)
                        .withRotationalRate(-driver.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
                )
            );


            new Trigger(() -> swerve.testConfigsChanged).onTrue(swerve.setDriveGains());

            
            driver.start().onTrue(swerve.runOnce(() -> swerve.resetPose(new Pose2d())));

            new Trigger(driver.rightTrigger()).onTrue(Commands.runOnce(() -> {Speedmodifier = 1.0;}));
            new Trigger(driver.rightTrigger()).onFalse(Commands.runOnce(() -> {Speedmodifier = 0.5;}));


        /*
        Turret Controls
        */
            // turret.setDefaultCommand(turret.runVoltsJoystick(() -> joystick.getRightX()));
            turret.setDefaultCommand(new TurretCameraPoseDefaultCommand(turret));

            operator.rightTrigger().onTrue(Commands.runOnce(() -> turret.currentTarget = Targets.Hub));
            operator.y().onTrue(Commands.runOnce(() -> turret.currentTarget = Targets.Feed));

            driver.rightBumper().whileTrue(turret.TurretRunWithVolts(Volts.of(0)));

            test.x().whileTrue(turret.TurretRunWithVolts(Volts.of(3))); //To the left
            test.b().whileTrue(turret.TurretRunWithVolts(Volts.of(-3))); //To the right

            // test.povDown().whileTrue(turret.resetEncoder());
            test.povDown().whileTrue(turret.TurretToSetpointCommand(Degrees.of(0)));


        /*
        Shooter Controls
        */
            new Trigger(() -> shooter.gainsChanged).whileTrue(shooter.updateGainsCommand());
            // driver.rightTrigger().whileTrue(shooter.runRPMCommand());
            operator.rightTrigger().whileTrue(shooter.treeMapRPMCommand());
            operator.y().whileTrue(shooter.runRPMCommand(3500));
            

        /*
        Hood Controls
        */
            hood.setDefaultCommand(Commands.run(() -> hood.runToPosition(), hood));
            // hood.setDefaultCommand(hood.treeMapRPMCommand());
            operator.rightTrigger().whileTrue(hood.treeMapRPMCommand());
            operator.y().whileTrue(hood.runToPositionCommand(15));

            joystick.y().whileTrue(hood.runVolts(2));
            joystick.a().whileTrue(hood.runVolts(-2));
        
        /*
        Feeder Controls
        */
            // operator.rightTrigger().whileTrue(feeder.runFeederVoltsCommand(12));
            // operator.L2().whileTrue(feeder.runFeederVoltsCommand(12));

            new Trigger(() -> shooter.atRPM).whileTrue(feeder.runFeederVoltsCommand(12));

        /*
        Spindexer Controls
        */
            // operator.rightTrigger().whileTrue(spindexer.runSpindexerVoltsCommand(12));
            // operator.L2().whileTrue(spindexer.runSpindexerVoltsCommand(12));

            new Trigger(() -> shooter.atRPM).whileTrue(spindexer.runSpindexerVoltsCommand(12));

            // operator.leftTrigger().whileTrue(spindexer.runSpindexerVoltsCommand(-12));


        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            swerve.applyRequest(() -> idle).ignoringDisable(true)
        );

        /*
        Intake
        */
            driver.leftBumper().whileTrue(intake.runVolts(10.56));
            driver.leftBumper().whileTrue(intake.runVolts(10.56));

            
        /*
        Intake Pivot
        */
            intakePivot.setDefaultCommand(intakePivot.runVoltsJoystick(() -> operator.getRightY()*0.5));

            operator.a().whileTrue(intakePivot.goDown());
            operator.x().whileTrue(intakePivot.bounce()).and(() -> intakePivot.up).whileTrue(intake.runVolts(6));
        

        swerve.registerTelemetry(logger::telemeterize);


    }

    public void printAutons(){
        SmartDashboard.putData("Auton", a_chooser);
        a_chooser.setDefaultOption("Do Nothing", 3);
        // a_chooser.addOption("test", 1);
        a_chooser.addOption("Left_Center (dont run yet unless you wanna yolo)", 2);
        a_chooser.addOption("Do Nothing", 3);
        a_chooser.addOption("Left_StopAtMiddle", 4);
        a_chooser.addOption("Right_StopAtMiddle", 5);
        a_chooser.addOption("Right_Center (dont run yet)", 6);
        a_chooser.addOption("Right_Middle2Cycle", 7);
        a_chooser.addOption("Left_Middle2Cycle", 8);

    }


    public Command getAutonomousCommand() {
        switch (a_chooser.getSelected()) {
            case 1:
                return new Test(swerve, robotState, shooter, intakePivot, intake, feeder, spindexer, hood, superstructure);

            case 2:
                return new Left_Center(swerve, robotState, shooter, intakePivot, intake, feeder, spindexer, hood, superstructure);

            case 3:
                return new doNOTHING(swerve);

            case 4:
                return new Left_StopAtMiddle(swerve, robotState, shooter, turret, intakePivot, intake, feeder, spindexer, hood, superstructure);

            case 5:
                return new Right_StopAtMiddle(swerve, robotState, shooter, turret, intakePivot, intake, feeder, spindexer, hood, superstructure);
                
            case 6:
                return new Right_Center(swerve, robotState, shooter, intakePivot, intake, feeder, spindexer, hood, superstructure);
            
            case 7:
                return new Right_Middle2Cycle(swerve, robotState, shooter, turret, intakePivot, intake, feeder, spindexer, hood, superstructure);
            case 8:
                return new Left_Middle2Cycle(swerve, robotState, shooter, turret, intakePivot, intake, feeder, spindexer, hood, superstructure);
                
            default:
                return new Test(swerve, robotState, shooter, intakePivot, intake, feeder, spindexer, hood, superstructure);

        }
    }  
}
