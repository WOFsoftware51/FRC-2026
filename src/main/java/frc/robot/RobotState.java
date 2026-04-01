package frc.robot;

import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;

import java.util.ArrayList;
import java.util.List;
import java.util.TreeMap;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class RobotState {
    private static RobotState instance = new RobotState();

    public static RobotState getInstance() {
        return instance;
    }
    
    private Pose2d pose2d = new Pose2d();

    private ChassisSpeeds robotChassisSpeeds = new ChassisSpeeds();
    private ChassisSpeeds fieldRelativeChassisSpeeds = new ChassisSpeeds();
    private Pose2d turretLimelightPose2d = new Pose2d();
    private Pose2d turretLimelightMegaTag2 = new Pose2d();

    private Pose2d chassisLimelightMegaTag2 = new Pose2d();

    private double hubToTurret;

    private Translation3d robotToTurreTranslation3d = 
        new Translation3d(
            Inches.of(-5.5), 
            Inches.of(-7.75), 
            Inches.of(16.09)
        );  
    
    private Transform3d robotToTurret = new Transform3d(
        robotToTurreTranslation3d,
        new Rotation3d(
            0,
            0,
            0
        )
    );

    private Pose3d rotatedTurret = new Pose3d();
    private Rotation3d rotatedAngle = new Rotation3d();

    private Transform3d turretToLimelight = new Transform3d(
        new Translation3d(  //TODO
            Inches.of(6.58), 
            Inches.of(0), 
            Inches.of(4.8)
        ),
        new Rotation3d(  //TODO
            Degrees.of(0), 
            Degrees.of(14.5), 
            Degrees.of(0)
        )
    );
    
    private Pose3d robotToLimelight = new Pose3d();

    private double mt1TimeStamp;
    private double mt2TimeStampTurret;

    private double mt2TimeStampChassis;


    InterpolatingDoubleTreeMap timeOfFlight = new InterpolatingDoubleTreeMap();

    private RobotState() {
        timeOfFlight.put(Inches.of(68.2).in(Meters), 0.55);
        timeOfFlight.put(Inches.of(87.4).in(Meters), 0.65);
        timeOfFlight.put(Inches.of(107.0).in(Meters), 0.75);
        timeOfFlight.put(Inches.of(127.0).in(Meters), 0.85);
        timeOfFlight.put(Inches.of(147.6).in(Meters), 0.95);
        timeOfFlight.put(Inches.of(166.6).in(Meters), 1.05);
        timeOfFlight.put(Inches.of(184.0).in(Meters), 1.15);
        timeOfFlight.put(Inches.of(208.0).in(Meters), 1.30);
    }


    public void setPose2d(Pose2d pose) {
        this.pose2d = pose;
    }
    
    public Pose2d getPose2d() {
        return this.pose2d;
    }   

    public Supplier<Pose2d> getPose2dSupplier() {
        return () -> this.pose2d;
    }   

    /**
     * Generates a circle of points from a Pose2d
     * 
     * @param pose Center Point
     * @param radius Distance from the center point to the edge
     * @param points Amount of Pose2d points to generate
     * 
     * @return A circle of Pose2ds
     */
    public List<Pose2d> generateCircle(Pose2d pose, double radius, int points) {
        List<Pose2d> circlePath = new ArrayList<>();
        if(points==0) {
            points = 1;
        } 
        double degreesOfCircle = 120;

        double allianceOffset = Constants.getAllianceColor().equals(Alliance.Red) ? 0 : Math.PI;


        for (int i = 0; i <= points; i++) {
            double angle = (Units.degreesToRadians(degreesOfCircle) * i / points) + allianceOffset -
                Units.degreesToRadians(((degreesOfCircle)/2)); 

            double x = pose.getX() + radius * Math.cos(angle);
            double y = pose.getY() + radius * Math.sin(angle);
            
            Rotation2d rotation = new Rotation2d(angle + Units.degreesToRadians(180));
            circlePath.add(new Pose2d(x, y, rotation));
        }
        return circlePath;
    }

    public Angle justinTurretAngle() {
        // Angle distance = Degrees.of(((-0.08 * Meters.of(getDistanceFromHubMeters()).in(Inches)) + 11));// + getPose2d().getRotation().getDegrees());

        Angle angle = Radians.of(Math.atan(-5.5 / (Meters.of(getDistanceFromHubMeters()).in(Inches) - 7)));
    
        Angle distance = Degrees.of(angle.in(Degrees) + getRobotToAllianceHubDegrees().in(Degrees));


        Logger.recordOutput("RobotState/angle", angle.in(Degrees));
        Logger.recordOutput("RobotState/justinTurretAngle", distance);

        return distance;
    }

    public Pose2d getNearestPoseFromHub() {
        List<Pose2d> poses = generateCircle(Constants.PoseConstants.kCurrentAllianceHubTarget.get(), 2.5, 0);
        Pose2d nearestPose = getPose2d().nearest(poses);

        Logger.recordOutput("Targeting/CirclePoses", poses.toArray(new Pose2d[0]));
    
        return nearestPose;
    }

    /**
     * Error from the front of the robot to the hub
     * 
     * @return Angle in Degrees
     */
    public Angle getRobotToAllianceHubDegrees() {
        double yError = Constants.PoseConstants.kCurrentAllianceHubTarget.get().getY() - getPose2d().getY();
        double xError = Constants.PoseConstants.kCurrentAllianceHubTarget.get().getX() - getPose2d().getX();
        Angle angleRadians = Radians.of(Math.atan2(yError,xError));
        double angleDegrees = angleRadians.in(Degree) - 90;
        return Degrees.of(angleDegrees);
    }
    public Angle getRobotToAllianceHubDegrees(Pose2d pose) {
        double yError = Constants.PoseConstants.kCurrentAllianceHubTarget.get().getY() - pose.getY();
        double xError = Constants.PoseConstants.kCurrentAllianceHubTarget.get().getX() - pose.getX();
        Angle angleRadians = Radians.of(Math.atan2(yError,xError));
        double angleDegrees = angleRadians.in(Degree) - 90;
        return Degrees.of(angleDegrees);
    }

    public Angle getTurretToAllianceHubDegrees() {
        
        Translation2d transform = new Translation2d(Inches.of(-7.75).in(Meters), Inches.of(5.5).in(Meters));
        Translation2d rotatedTransform = transform.rotateBy(getPose2d().getRotation());

        double yError = Constants.PoseConstants.kCurrentAllianceHubTarget.get().getY() - (getPose2d().getY());
        double xError = Constants.PoseConstants.kCurrentAllianceHubTarget.get().getX() - (getPose2d().getX());

        Angle angleRadians = Radians.of(Math.atan2(yError,xError));
        double angleDegrees = angleRadians.in(Degree)+90;

        Logger.recordOutput("RobotState/getTurretToAlluanceHubDegrees", angleDegrees);
        Logger.recordOutput("RobotState/rotatedTransformY", new Pose2d(rotatedTransform.getX(), rotatedTransform.getY(), new Rotation2d()));

        return Degrees.of(angleDegrees);
    }

    public void setChassisSpeeds(ChassisSpeeds robotChassisSpeeds, ChassisSpeeds fieldRelativeChassisSpeeds) {
        this.robotChassisSpeeds = robotChassisSpeeds;
        this.fieldRelativeChassisSpeeds = fieldRelativeChassisSpeeds;
    }
    public ChassisSpeeds getChassisSpeeds() {
        return robotChassisSpeeds;
    }

    public ChassisSpeeds getFieldRelativeChassisSpeeds() {
        return fieldRelativeChassisSpeeds;
    }

    public void setTurretLimelightPose2d(Pose2d pose2d) {
        this.turretLimelightPose2d = pose2d;
    }
    public Pose2d getTurretLimelightPose2d() {
        return turretLimelightPose2d;
    }

    public void setChassisLimelightPose2d(Pose2d pose2d) {
        this.chassisLimelightMegaTag2 = pose2d;
    }
    public Pose2d getChassisLimelightPose2d() {
        return chassisLimelightMegaTag2;
    }

    
    // InterpolatingTreeMap<Double, Rotation2d> turretEncoderAtTimeStamp;;

    // public void getTurretEncoderAtTimeStamp(double timeStamp, double angle) {
    //     turretEncoderAtTimeStamp.put(timeStamp, new Rotation2d(Units.degreesToRadians(angle)));
    //     turretEncoderAtTimeStamp.clear();
    // }

    TreeMap<Double, Rotation2d> turretTimeStamp = new TreeMap<>();
    public void setTurretTimeStamp(double timeStamp, double angle) {
        turretTimeStamp.put(timeStamp, new Rotation2d(Units.degreesToRadians(angle)));
        turretTimeStamp.headMap(timeStamp-1).clear();
    }

    public double getTurretTimeStamp(double timeStamp) {  
        var floor = turretTimeStamp.floorEntry(timeStamp);
        var ceiling = turretTimeStamp.ceilingEntry(timeStamp);
        
        if(floor == null && ceiling == null) {
            return 0;
        }
        else if(floor == null) {
            return ceiling.getValue().getDegrees();
        }
        else if(ceiling == null) {
            return floor.getValue().getDegrees();
        }

        double timeDif = ceiling.getKey() - floor.getKey();
        if(timeDif <= Double.MIN_VALUE) return floor.getValue().getDegrees();

        double percentDone = (timeStamp - floor.getValue().getDegrees()) / timeDif;

        return MathUtil.interpolate(floor.getValue().getDegrees(), ceiling.getValue().getDegrees(), percentDone);
    }


    public void setTurretLimelightMegaTag2(Pose2d pose2d) {
        this.turretLimelightMegaTag2 = pose2d;
    }
    public Pose2d getTurretLimelightMegaTag2() {
        return turretLimelightMegaTag2;
    }

    public void setLimelightTurretTimeStamp(double mt1Latency, double mt2Latency) {
        this.mt1TimeStamp = mt1Latency;
        this.mt2TimeStampTurret = mt2Latency;
    }
    public double getMegaTag1TimeStamp() {
        return mt1TimeStamp;
    }
    public double getMegaTag2TimeStampTurret() {
        return mt2TimeStampTurret;
    }

    public void setLimelightChassisTimeStamp(double mt2Latency) {
        this.mt2TimeStampChassis = mt2Latency;
    }
    public double getMegaTag2TimeStampChassis() {
        return mt2TimeStampChassis;
    }

    public void setRobotToTurret(double turretYawDegrees) {
        robotToTurret = new Transform3d(
            robotToTurreTranslation3d,
            new Rotation3d(  //TODO
                Degrees.of(0), 
                Degrees.of(0), 
                Degrees.of(90)
            )
        );

        rotatedAngle = new Rotation3d(Degrees.of(0), Degrees.of(0), Degrees.of(turretYawDegrees));
        rotatedTurret = new Pose3d(robotToTurret.getTranslation(), robotToTurret.getRotation().plus(rotatedAngle));

    }

    public Pose3d getRobotToTurret() {
        return rotatedTurret;
    }

    public Transform3d getTurretToLimelight(){
        // var a = new Pose3d(turretToLimelight.getTranslation(), turretToLimelight.getRotation()).rotateBy(rotatedAngle);
        // var done = new Transform3d(a.getTranslation(), a.getRotation());
        return turretToLimelight;
    }

    
    public void setRobotToLimelight() {
        robotToLimelight = getRobotToTurret().plus(getTurretToLimelight());
    }

    public Pose3d getRobotToLimelight() {
        return robotToLimelight;
    }

    public double getDistanceFromHubMeters() {
        double yError = Constants.PoseConstants.kCurrentAllianceHubTarget.get().getY() - getPose2d().getY();
        double xError = Constants.PoseConstants.kCurrentAllianceHubTarget.get().getX() - getPose2d().getX();

        double distance = 
            Math.sqrt(
                Math.pow(yError, 2) +
                Math.pow(xError, 2)
            );
            
        Logger.recordOutput("RobotState/getDistanceFromHubMeters", distance);

        return distance;
    }

    public void setTurretToHub(double distance) {
        this.hubToTurret = distance;
    }

    public double getTurretToHub() {
        return this.hubToTurret;
    }

    
    public double getTimeOfFlight() {
        return timeOfFlight.get(getTurretToHub());
    }
}
