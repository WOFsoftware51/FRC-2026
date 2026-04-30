package frc.robot;

import java.io.IOException;

import org.json.simple.parser.ParseException;

import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FileVersionException;

public class PathPlannerPaths {
    public PathPlannerPath RightTrench_Center6;
    public PathPlannerPath RightPickUp_RightTrench6;
    public PathPlannerPath RightTrench_Center;
    public PathPlannerPath RightTrench_Center2;
    public PathPlannerPath RightCenter_Pickup2;
    public PathPlannerPath RightCenter_Pickup26;
    public PathPlannerPath RightPickUp_RightTrench2;
    public PathPlannerPath emptyRightTrench;
    public PathPlannerPath emptyLeftTrench6;
    public PathPlannerPath LeftTrench_Center6;
    public PathPlannerPath LeftCenter_Pickup2;
    public PathPlannerPath LeftPickUp_LeftTrenchMoreCenter;
    public PathPlannerPath LeftTrenchMoreCenter_LeftCenter2;
    public PathPlannerPath LeftCenter_Pickup26;
    public PathPlannerPath RightPickUp_RightTrenchMoreCenter;
    public PathPlannerPath RightPickUp_RightTrenchMoreCenter6;
    public PathPlannerPath LeftPickUp_LeftTrenchMoreCenter6;
    public PathPlannerPath RightTrenchMoreCenter_Center2;
    public PathPlannerPath LeftPickUp_LeftTrench26;
    public PathPlannerPath LeftTrench_Center26;
    public PathPlannerPath LeftPickUp_LeftTrench6;
    public PathPlannerPath Middle_Depot;
    public PathPlannerPath Depot_Shoot;

    public PathPlannerPaths() {
        try {
            RightTrench_Center6 = PathPlannerPath.fromPathFile("RightTrench_Center6");
            RightTrench_Center = PathPlannerPath.fromPathFile("RightTrench_Center");
            RightPickUp_RightTrench6 = PathPlannerPath.fromPathFile("RightPickUp_RightTrench6");
            RightTrench_Center2 = PathPlannerPath.fromPathFile("RightTrench_Center2");
            RightCenter_Pickup2 = PathPlannerPath.fromPathFile("RightCenter_Pickup2");
            RightCenter_Pickup26 = PathPlannerPath.fromPathFile("RightCenter_Pickup26");
            RightPickUp_RightTrench2 = PathPlannerPath.fromPathFile("RightPickUp_RightTrench2");
            emptyRightTrench = PathPlannerPath.fromPathFile("emptyRightTrench");
            emptyLeftTrench6 = PathPlannerPath.fromPathFile("emptyLeftTrench6");
            LeftTrench_Center6 = PathPlannerPath.fromPathFile("LeftTrench_Center6");
            LeftCenter_Pickup2 = PathPlannerPath.fromPathFile("LeftCenter_Pickup2");
            LeftPickUp_LeftTrenchMoreCenter = PathPlannerPath.fromPathFile("LeftPickUp_LeftTrenchMoreCenter");
            LeftTrenchMoreCenter_LeftCenter2 = PathPlannerPath.fromPathFile("LeftTrenchMoreCenter_LeftCenter2");
            LeftCenter_Pickup26 = PathPlannerPath.fromPathFile("LeftCenter_Pickup26");
            RightPickUp_RightTrenchMoreCenter = PathPlannerPath.fromPathFile("RightPickUp_RightTrenchMoreCenter");
            RightPickUp_RightTrenchMoreCenter6 = PathPlannerPath.fromPathFile("RightPickUp_RightTrenchMoreCenter6");
            LeftPickUp_LeftTrenchMoreCenter6 = PathPlannerPath.fromPathFile("LeftPickUp_LeftTrenchMoreCenter6");
            RightTrenchMoreCenter_Center2 = PathPlannerPath.fromPathFile("RightTrenchMoreCenter_Center2");
            LeftPickUp_LeftTrench26 = PathPlannerPath.fromPathFile("LeftPickUp_LeftTrench26");
            LeftTrench_Center26 = PathPlannerPath.fromPathFile("LeftTrench_Center26");
            LeftPickUp_LeftTrench6 = PathPlannerPath.fromPathFile("LeftPickUp_LeftTrench6");
            Middle_Depot = PathPlannerPath.fromPathFile("Middle_Depot");
            Depot_Shoot = PathPlannerPath.fromPathFile("Depot_Shoot");
        }
        catch(Exception e) {
            e.printStackTrace();
        }
    }
}
