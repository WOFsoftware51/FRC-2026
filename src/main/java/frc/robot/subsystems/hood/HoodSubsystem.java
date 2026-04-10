package frc.robot.subsystems.hood;

import static edu.wpi.first.units.Units.*;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotState;
import frc.robot.RobotState.Targets;
import frc.robot.util.LoggedTunableNumber;

public class HoodSubsystem extends SubsystemBase{
    HoodIO io;
    HoodIOInputsAutoLogged inputs = new HoodIOInputsAutoLogged();
    RobotState robotState;

    LoggedTunableNumber position = new LoggedTunableNumber("Hood/SetDegrees", 0.0);

    InterpolatingDoubleTreeMap treeMap = new InterpolatingDoubleTreeMap();
    private InterpolatingDoubleTreeMap treeMapFeed = new InterpolatingDoubleTreeMap();

    public double currentTarget;

    public HoodSubsystem(HoodIO io, RobotState robotState) {
        this.io = io;
        this.robotState = robotState;

        treeMap.put(Inches.of(68.2).in(Meters), 2.0);
        treeMap.put(Inches.of(87.4).in(Meters), 2.0);
        treeMap.put(Inches.of(107.0).in(Meters), 3.0);
        treeMap.put(Inches.of(127.0).in(Meters), 3.0);
        treeMap.put(Inches.of(147.6).in(Meters), 4.0);
        treeMap.put(Inches.of(166.6).in(Meters), 8.0);
        treeMap.put(Inches.of(184.0).in(Meters), 10.0);
        treeMap.put(Inches.of(208.0).in(Meters), 12.0);

        treeMapFeed.put(Meters.of(4.03).in(Meters), 10.0);
        treeMapFeed.put(Meters.of(5.70).in(Meters), 15.0);
        treeMapFeed.put(Meters.of(7.03).in(Meters), 16.0);

    }

    public void runToDistanceFromHub() {
        io.runSetpoint(Degrees.of(treeMap.get(robotState.getDistanceFromHubMeters())));
    }
    
    public void runToPosition() {
        io.runSetpoint(Degrees.of(position.get()));
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Hood", inputs);


        if(RobotState.getInstance().getCurrentTarget() == Targets.Hub) {
            currentTarget = treeMap.get(Double.valueOf(RobotState.getInstance().getTurretToHub()));

        }
        else if(RobotState.getInstance().getCurrentTarget() == Targets.Feed) {
            currentTarget = treeMapFeed.get(Double.valueOf(RobotState.getInstance().getTurretToHub()));
        }

        Logger.recordOutput("Hood/TreeMap Angle", currentTarget);
    }

    public Command runVolts(double volts) {
        return run(() ->
            io.runVolts(Volts.of(volts))
        )
        .finallyDo(() ->
            io.stop()
        );
    }

    public Command resetEncoder() {
        return runOnce(() ->
            io.resetEncoder()
        );
    }

    public Command runToPositionCommand(double degrees) {
        return run(() ->
            io.runSetpoint(Degrees.of(degrees))
        )
        .finallyDo(() ->
            io.stop()
        );
    }

    public Command treeMapRPMCommand() {
        return run(() ->
            // io.runSetpoint(Degrees.of(treeMap.get(Double.valueOf(robotState.getTurretToHub()))))
            io.runSetpoint(Degrees.of(currentTarget))
        );
    
    }


}
