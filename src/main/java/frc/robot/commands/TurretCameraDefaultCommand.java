// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.turret.TurretSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class TurretCameraDefaultCommand extends Command {
  /** Creates a new TurretDefaultCommand. */
  private TurretSubsystem turret;
  private boolean isSearching;
  private boolean closerToRight;

  private final double searchTolerance = 10;
  

  private double timeWithoutSeeingTagSeconds = 0.0;
  private double currentTime = 0.0;
  private double lastSeenTimeStamp = 0.0;

  public TurretCameraDefaultCommand(TurretSubsystem m_turret) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.turret = m_turret;

    addRequirements(m_turret);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    isSearching = false;
    closerToRight = false;
    timeWithoutSeeingTagSeconds = 0.0;
    lastSeenTimeStamp = Timer.getFPGATimestamp();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    currentTime = Timer.getFPGATimestamp();

    if(!turret.tv) { 
      timeWithoutSeeingTagSeconds = currentTime - lastSeenTimeStamp;
    }
    else {
      isSearching = false;
      lastSeenTimeStamp = Timer.getFPGATimestamp(); 
      currentTime = lastSeenTimeStamp;
      timeWithoutSeeingTagSeconds = 0.0;
    }

    if(timeWithoutSeeingTagSeconds>=0.5) {
      isSearching = true;
    }


    
    if(isSearching) {
      if(closerToRight) {
        // turret.runToSetpoint(Degrees.of(Constants.TurretConstants.kForwardLimit));
        turret.run(Volts.of(3.5));
        if((Constants.TurretConstants.kForwardLimit - turret.currentDegrees) < searchTolerance) {
          closerToRight = false;
        }

      }
      else {
      // turret.runToSetpoint(Degrees.of(Constants.TurretConstants.kReverseLimit));
        turret.run(Volts.of(-3.5));
        if((Constants.TurretConstants.kReverseLimit - turret.currentDegrees) > -searchTolerance) {
          closerToRight = true;
        }

      }

    }
    else {
      turret.turretCameraAimToHub();

      if((turret.currentDegrees) >= 0) {
        closerToRight = true;
      }
      else {
        closerToRight = false;
      }

    }


  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
