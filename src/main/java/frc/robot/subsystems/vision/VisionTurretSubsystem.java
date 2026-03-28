// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotState;

public class VisionTurretSubsystem extends SubsystemBase {
  private VisionIO io;
  public VisionIOInputsAutoLogged inputs = new VisionIOInputsAutoLogged();
  Pose3d cameraPose2d;
  Pose2d visionPose = new Pose2d();
  Pose2d visionMegaTag2 = new Pose2d();

  /** Creates a new VisionTurretSubsystem. */
  public VisionTurretSubsystem(VisionIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    
    if(inputs.tv) {
      visionPose = new Pose2d(
        inputs.botpose_x,
        inputs.botpose_y,
        new Rotation2d(inputs.botpose_rot)
      );

      Logger.recordOutput("VisionTurret/botpose", 
        new double[] {
          inputs.botpose_x, 
          inputs.botpose_y,
          inputs.botpose_rot
        }
      );

      visionMegaTag2 = new Pose2d(
        inputs.MegaTag2_x,
        inputs.MegaTag2_y,
        new Rotation2d(inputs.MegaTag2_rot)
      );

      Logger.recordOutput("VisionTurret/botposeMegaTag2", 
        new double[] {
          inputs.MegaTag2_x, 
          inputs.MegaTag2_y,
          inputs.MegaTag2_rot
        }
      );

    }


    cameraPose2d = LimelightHelpers.getBotPose3d_wpiBlue(Constants.VisionConstants.kTurretLimelight);
    cameraPose2d.transformBy(new Transform3d(RobotState.getInstance().getRobotToLimelight().getTranslation(), RobotState.getInstance().getRobotToLimelight().getRotation()).inverse());

    if(inputs.tv){
      RobotState.getInstance().setTurretLimelightPose2d(visionPose);
      RobotState.getInstance().setTurretLimelightMegaTag2(visionMegaTag2);
    }
    RobotState.getInstance().setLimelightTurretTimeStamp(inputs.mt1TimeStamp, inputs.mt2TimeStamp);

    Logger.processInputs("Vision/Turret Limelight", inputs);
  }
}
