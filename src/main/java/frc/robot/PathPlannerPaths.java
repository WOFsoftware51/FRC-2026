package frc.robot;

import java.io.IOException;

import org.json.simple.parser.ParseException;

import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FileVersionException;

public class PathPlannerPaths {
    public PathPlannerPath RightTrench_Center;
    public PathPlannerPath RightPickUp_RightTrench;
    public PathPlannerPath RightTrench_Center2;
    public PathPlannerPath RightCenter_Pickup2;
    public PathPlannerPath RightPickUp_RightTrench2;
    public PathPlannerPath emptyRightTrench;

    public PathPlannerPaths() {
        try {
            RightTrench_Center = PathPlannerPath.fromPathFile("RightTrench_Center");
            // RightCenter_Pickup = PathPlannerPath.fromPathFile("RightCenter_Pickup");
            RightPickUp_RightTrench = PathPlannerPath.fromPathFile("RightPickUp_RightTrench");
            RightTrench_Center2 = PathPlannerPath.fromPathFile("RightTrench_Center2");
            RightCenter_Pickup2 = PathPlannerPath.fromPathFile("RightCenter_Pickup2");
            RightPickUp_RightTrench2 = PathPlannerPath.fromPathFile("RightPickUp_RightTrench2");
            emptyRightTrench = PathPlannerPath.fromPathFile("emptyRightTrench");
        }
        catch(Exception e) {
            e.printStackTrace();
        }
    }
}
