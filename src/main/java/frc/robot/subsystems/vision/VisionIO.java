package frc.robot.subsystems.vision;

import java.lang.reflect.Array;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.DoubleArrayEntry;


public interface VisionIO {

    @AutoLog
    class VisionIOInputs {
        public double tx = 0;
        public double ty = 0;
        public boolean tv = false;
        public double ta = 0;

        public double tl;
        public double cl;

        public double[] hw;

        // public Pose2d botpose = new Pose2d();

        public double botpose_x = 0.0;
        public double botpose_y = 0.0;
        public double botpose_rot = 0.0; // Radians

        public double MegaTag2_x = 0.0;
        public double MegaTag2_y = 0.0;
        public double MegaTag2_rot = 0.0; // Radians

        public double mt1TimeStamp;
        public double mt2TimeStamp;

        
        public double yaw;
        public double roll;
        public double pitch;
        
        public double currentPipeline;

    }

    void updateInputs(VisionIOInputs inputs);
    void resetYaw();
}
