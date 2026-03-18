package frc.robot.subsystems.vision;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.RobotState;

public class VisionIOSim implements VisionIO {
    VisionSystemSim visionSim = new VisionSystemSim("Turret");
    AprilTagFieldLayout tagLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

    SimCameraProperties cameraProperties = new SimCameraProperties();
    
    PhotonCamera camera = new PhotonCamera("Turret");
    PhotonCameraSim cameraSim = new PhotonCameraSim(camera, cameraProperties);

    PhotonPoseEstimator poseEstimator = new PhotonPoseEstimator(
        tagLayout, 
        RobotState.getInstance().getRobotToLimelight()
    );

    public VisionIOSim() {
        cameraProperties.setAvgLatencyMs(30);
        cameraProperties.setLatencyStdDevMs(5);
        cameraProperties.setCalibration(640, 480, new Rotation2d(82, 56.2));
        cameraProperties.setFPS(50);
        
        visionSim.addAprilTags(tagLayout);
        visionSim.addCamera(cameraSim, RobotState.getInstance().getRobotToLimelight());

        poseEstimator.setPrimaryStrategy(PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR);
    }
    
    @Override
    public void updateInputs(VisionIOInputs inputs) {
        visionSim.update(RobotState.getInstance().getPose2d());

        visionSim.adjustCamera(cameraSim, RobotState.getInstance().getRobotToLimelight());
        poseEstimator.setRobotToCameraTransform(RobotState.getInstance().getRobotToLimelight());

        PhotonPipelineResult result = camera.getLatestResult();
        Optional<EstimatedRobotPose> estimatedPose = poseEstimator.update(result);
        
        inputs.tv = result.hasTargets();

        if (inputs.tv && estimatedPose.isPresent()) {
            Pose2d rawPose = estimatedPose.get().estimatedPose.toPose2d();
            
            if (Double.isFinite(rawPose.getX()) && Double.isFinite(rawPose.getY()) && Double.isFinite(rawPose.getRotation().getRadians())) {
                inputs.botpose_x = rawPose.getX();
                inputs.botpose_y = rawPose.getY();
                inputs.botpose_rot = rawPose.getRotation().getRadians();
            } 
        } 


        // Pose2d pose = estimatedPose.isPresent() ? estimatedPose.get().estimatedPose.toPose2d() : new Pose2d();
        // boolean isBad = 
        //     Double.isNaN(pose.getX()) || 
        //     Double.isNaN(pose.getY()) || 
        //     Double.isNaN(pose.getRotation().getRadians()) ||
        //     Double.isInfinite(pose.getX()) || 
        //     Double.isInfinite(pose.getY()) || 
        //     Double.isInfinite(pose.getRotation().getRadians());
        // inputs.botpose = isBad ? new Pose2d() : pose;
        
    }

    @Override
    public void resetYaw() {
        // TODO Auto-generated method stub
    }
    
}
