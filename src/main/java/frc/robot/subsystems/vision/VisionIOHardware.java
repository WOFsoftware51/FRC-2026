package frc.robot.subsystems.vision;

import edu.wpi.first.networktables.DoubleArrayEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.subsystems.vision.LimelightHelpers.IMUData;

public class VisionIOHardware implements VisionIO {
    public String limelight;

    public VisionIOHardware(String limelightName) {
        this.limelight = limelightName;
    }

    @Override
    public void updateInputs(VisionIOInputs inputs) {
        inputs.tx = LimelightHelpers.getTX(limelight);
        inputs.ty = LimelightHelpers.getTY(limelight);
        inputs.tv = LimelightHelpers.getTV(limelight);
        inputs.ta = LimelightHelpers.getTA(limelight);

        inputs.tl = LimelightHelpers.getLatency_Pipeline(limelight);
        inputs.cl = LimelightHelpers.getLatency_Capture(limelight);

        inputs.hw =  NetworkTableInstance.getDefault().getTable("limelight").getEntry("hw").getDoubleArray(new double[0]);



        // Pose2d rawPose = LimelightHelpers.getBotPose2d_wpiBlue(limelight);
        // Pose2d rawMegaTag2Pose = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelight).pose;
        LimelightHelpers.PoseEstimate rawPose = LimelightHelpers.getBotPoseEstimate_wpiBlue(limelight);
        LimelightHelpers.PoseEstimate rawMegaTag2Pose = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelight);
        
        inputs.mt1TimeStamp = rawPose.timestampSeconds;
        inputs.mt2TimeStamp = rawMegaTag2Pose.timestampSeconds;
        
        if (rawPose.tagCount > 0 || rawMegaTag2Pose.tagCount > 0) {
            
            if (
                Double.isFinite(rawPose.pose.getX()) && 
                Double.isFinite(rawPose.pose.getY()) && 
                Double.isFinite(rawPose.pose.getRotation().getRadians())
            )   {  
                    inputs.botpose_x = rawPose.pose.getX();
                    inputs.botpose_y = rawPose.pose.getY();
                    inputs.botpose_rot = rawPose.pose.getRotation().getRadians();
            } 
        
            if (
                Double.isFinite(rawMegaTag2Pose.pose.getX()) && 
                Double.isFinite(rawMegaTag2Pose.pose.getY()) && 
                Double.isFinite(rawMegaTag2Pose.pose.getRotation().getRadians())
            )   {
                    inputs.MegaTag2_x = rawMegaTag2Pose.pose.getX();
                    inputs.MegaTag2_y = rawMegaTag2Pose.pose.getY();
                    inputs.MegaTag2_rot = rawMegaTag2Pose.pose.getRotation().getRadians();
            }

        }
        
        IMUData imu = LimelightHelpers.getIMUData(limelight);
        inputs.yaw = imu.Yaw;
        inputs.roll = imu.Roll;
        inputs.pitch = imu.Pitch;

        inputs.currentPipeline = LimelightHelpers.getCurrentPipelineIndex(limelight);
    }

    @Override
    public void resetYaw() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'resetYaw'");
    }

    
}
