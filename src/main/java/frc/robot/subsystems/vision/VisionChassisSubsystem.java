package frc.robot.subsystems.vision;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotState;

public class VisionChassisSubsystem extends SubsystemBase{
    private VisionIO io;
    public VisionIOInputsAutoLogged inputs = new VisionIOInputsAutoLogged();

    public Pose3d cameraPose2d;
    Pose2d visionPose = new Pose2d();
    Pose2d visionMegaTag2 = new Pose2d();

    public VisionChassisSubsystem(VisionIO io) {
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

            Logger.recordOutput("VisionChassis/botpose", 
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

            Logger.recordOutput("VisionChassis/botposeMegaTag2", 
                new double[] {
                inputs.MegaTag2_x, 
                inputs.MegaTag2_y,
                inputs.MegaTag2_rot
                }
            );

        }

        cameraPose2d = LimelightHelpers.getBotPose3d_wpiBlue(Constants.VisionConstants.kChassisLimelight);

        RobotState.getInstance().setChassisLimelightPose2d(visionMegaTag2);
        RobotState.getInstance().setLimelightChassisTimeStamp(inputs.mt2TimeStamp);
        Logger.processInputs("Vision/Chassis Limelight", inputs);
    }
}