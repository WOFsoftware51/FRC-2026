// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Autons;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.factories.Superstructure;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Right_StopAtMiddle extends SequentialCommandGroup {

  public Right_StopAtMiddle(Superstructure superstructure) {
    PathPlannerPath RightTrench_Center;


    try {
      RightTrench_Center = PathPlannerPath.fromPathFile("RightTrench_Center");

      addCommands(
        AutoBuilder.resetOdom(RightTrench_Center.getStartingHolonomicPose().get()), 
        superstructure.test(),
        AutoBuilder.followPath(RightTrench_Center)
      );

    }
    catch(Exception e) {
      e.printStackTrace();
    }
  }
}
