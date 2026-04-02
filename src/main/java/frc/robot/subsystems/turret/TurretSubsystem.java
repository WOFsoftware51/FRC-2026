// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.turret;

import static edu.wpi.first.units.Units.*;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotState;
import frc.robot.subsystems.vision.VisionTurretSubsystem;
import frc.robot.util.LoggedTunableNumber;

public class TurretSubsystem extends SubsystemBase {
  private final TurretIO io;
  private final VisionTurretSubsystem limelight;
  private RobotState robotState;
  private final TurretIOInputsAutoLogged inputs = new TurretIOInputsAutoLogged();

  public double tx;
  public boolean tv;
  public double currentDegrees;
  public double targetDegrees;

  public double robotHeading;
  public double degreesToHub;

  private double visionLatency;
  public double totalLatency;
  
  private boolean inTXWindow = false;
  
  double chassisOmegaDPS;

  // private Pose2d turretPose2d = new Pose2d();

  LoggedTunableNumber mainThreadLatency = new LoggedTunableNumber("mainThreadLatency", 0.07);

  public static enum Targets {
    Hub, 
    Feed
  }

  public static Targets currentTarget = Targets.Hub;

  public TurretSubsystem(TurretIO io, VisionTurretSubsystem limelight, RobotState robotState) {
    this.io = io;
    this.limelight = limelight;
    this.robotState = robotState;
  }



  private double turretVelX, turretVelY;

  private void updateTurretVelocity() {
    double vx = robotState.getChassisSpeeds().vxMetersPerSecond;
    double vy = robotState.getChassisSpeeds().vyMetersPerSecond;
    double angle = Math.toRadians((-currentDegrees + 90));
    double cos = Math.cos(angle);
    double sin = Math.sin(angle);
    turretVelX = vx * cos - vy * sin;
    turretVelY = vx * sin + vy * cos;
  }

  // public void runToSetpoint(Angle degrees) {
  //   io.runSetpoint(degrees);
  // }


  public void run(Voltage volts) {
    io.runVolts(volts);
  }

  public void turretCameraAimToHub() {
    // if(Math.abs(targetDegrees-currentDegrees) > 2)
      io.runSetpoint(Degrees.of(targetDegrees));
    // else {
      // io.stop();
    // }
  }


  public Command resetEncoder() {
    return runOnce(
      ()-> io.resetEncoder()
    );
  }  

  
  private double turretFieldX, turretFieldY;

  private void updateTurretFieldPose() {
      double robotX = robotState.getPose2d().getX();
      double robotY = robotState.getPose2d().getY();
      double headingRad = Math.toRadians(robotHeading);
      double cos = Math.cos(headingRad);
      double sin = Math.sin(headingRad);
      double tx = robotState.getRobotToTurret().getX();
      double ty = robotState.getRobotToTurret().getY();
      turretFieldX = robotX + tx * cos - ty * sin;
      turretFieldY = robotY + tx * sin + ty * cos;
  }

  private Angle getTurretToSetpointAngle(Pose2d setpoint) {
    double yError = setpoint.getY() - turretFieldY;
    double xError = setpoint.getX() - turretFieldX;
    Angle angleRadians = Radians.of(Math.atan2(yError,xError));
    double angleDegrees = angleRadians.in(Degree) - 90;
    return Degrees.of(angleDegrees);

  }

  private Angle getTurretToSetpointAngle() {
    double yError = Constants.PoseConstants.kStationTargetRed.getY() - turretFieldY;
    double xError = Constants.PoseConstants.kStationTargetRed.getX() - turretFieldX;
    Angle angleRadians = Radians.of(Math.atan2(yError,xError));
    double angleDegrees = angleRadians.in(Degree) - 90;
    return Degrees.of(angleDegrees);

  }

  @Override
  public void periodic() {
    this.io.updateInputs(inputs);
    robotHeading = robotState.getPose2d().getRotation().getDegrees();
    robotState.setRobotToTurret(inputs.position.in(Degrees));

    
    updateTurretVelocity();
    updateTurretFieldPose();

    // turretPose2d = robotState.getPose2d().transformBy(new Transform2d(robotState.getRobotToTurret().toPose2d().getTranslation(), robotState.getRobotToTurret().toPose2d().getRotation()));

    // double turretToHubDistance = turretPose2d.getTranslation().getDistance(Constants.PoseConstants.kCurrentAllianceHubTarget.get().getTranslation());
    double turretToHubDistance = 
      Math.hypot(
        Constants.PoseConstants.kCurrentAllianceHubTarget.get().getTranslation().getX() - 
        turretFieldX, 
        Constants.PoseConstants.kCurrentAllianceHubTarget.get().getTranslation().getY()- 
        turretFieldY
      );
    double turretToRightStation = 
      Math.hypot(
        Constants.PoseConstants.kCurrentAllianceHubTarget.get().getTranslation().getX()- 
        turretFieldX, 
        Constants.PoseConstants.kCurrentAllianceHubTarget.get().getTranslation().getY()- 
        turretFieldY
      );

    if(currentTarget == Targets.Hub){
      robotState.setTurretToHub(turretToHubDistance);
    }
    else if(currentTarget == Targets.Feed) {
      robotState.setTurretToHub(turretToRightStation);
    }
    else {
      robotState.setTurretToHub(turretToHubDistance);
    }
    
    robotState.setRobotToLimelight();
    
    visionLatency = ((limelight.inputs.tl + limelight.inputs.cl)/1000.0);
    totalLatency = visionLatency + mainThreadLatency.getAsDouble();

    // Pose3d turretPose3d = new Pose3d(turretPose2d);
    // Logger.recordOutput("turretPose3d", 
    //   new double[] {
    //     turretPose3d.getX(), 
    //     turretPose3d.getY(), 
    //     turretPose3d.getZ(),
    //     turretPose3d.getRotation().getZ()
    //   }
    // );



    // degreesToHub = robotState.getRobotToAllianceHubDegrees(turretPose2d).in(Degrees);
    degreesToHub = robotState.getRobotToAllianceHubDegrees(turretFieldX, turretFieldY).in(Degrees);


    if(Math.abs(limelight.inputs.tx)>0.5) {
      tx = limelight.inputs.tx;
      inTXWindow = false;
    }
    else if(Math.abs(limelight.inputs.tx)<0.3) {
      tx = 0;
      inTXWindow = true;
    }


    tv = limelight.inputs.tv;
    chassisOmegaDPS = Math.toDegrees(robotState.getChassisSpeeds().omegaRadiansPerSecond);
    
    
    currentDegrees = MathUtil.inputModulus(inputs.position.in(Degrees), -80, 280);
        
    // targetDegrees = (currentDegrees - tx) + (
    //   chassisOmegaDPS * totalLatency);

    double angleMovingOffset = (chassisOmegaDPS * totalLatency);

    if(currentTarget == Targets.Hub){
      targetDegrees = MathUtil.inputModulus(degreesToHub - robotHeading, -80, 280) 
        - (angleMovingOffset)
        // + Units.radiansToDegrees(Math.atan((robotState.getTimeOfFlight()*turretVelY)/robotState.getTurretToHub()))
      ;
    }
    else if(currentTarget == Targets.Feed) {
      targetDegrees = MathUtil.inputModulus(getTurretToSetpointAngle().in(Degree) - robotHeading, -80, 280) 
        - (angleMovingOffset)
      ;
    }
    else {
      targetDegrees = MathUtil.inputModulus(degreesToHub - robotHeading, -80, 280) 
        - (angleMovingOffset)
        // + Units.radiansToDegrees(Math.atan((robotState.getTimeOfFlight()*turretVelY)/robotState.getTurretToHub()))
      ;
    }
    
    Logger.recordOutput("turretToHubDistance", turretToHubDistance);


    robotState.setTurretTimeStamp(Timer.getFPGATimestamp(), currentDegrees);


    Logger.recordOutput("turretPose2d", 
      new double[] {
        turretFieldX, 
        turretFieldY, 
        currentDegrees + robotHeading + 90
      }
    );

    Logger.processInputs("Turret", inputs);
    Logger.recordOutput("Turret/currentDegrees", currentDegrees);
    Logger.recordOutput("Turret/targetDegrees", targetDegrees);
    Logger.recordOutput("Turret/inTXWindow", inTXWindow);

    // Logger.recordOutput("Turret/RobotToLimelight", robotState.getLimelightTransformPose());

    Logger.recordOutput("Turret/angleMovingOffset", angleMovingOffset);

    Logger.recordOutput("Turret/HubError", degreesToHub);
    
    
  }
    
    
  public Command runVoltsJoystick(DoubleSupplier volts) {
    return run(() -> 
      io.runVolts(Volts.of(volts.getAsDouble()*12))
    )
    .finallyDo(() ->
      io.stop()
    );
  }



  public Command TurretRunWithVolts(Voltage speedInVolts) {
    return run(() ->
      this.io.runVolts(speedInVolts)
    )
    .finallyDo(
      () -> io.stop()
    );
  }
  
  public Command TurretToSetpointCommand(Angle positionInDegrees) {
    return run(() ->
      io.runSetpoint(positionInDegrees)
    )
    .finallyDo(() -> 
      io.stop()
    );
  }

  

}
