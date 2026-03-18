package frc.robot.subsystems.pivot;

import static edu.wpi.first.units.Units.*;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotState;
import frc.robot.util.LoggedTunableNumber;

public class PivotSubsystem extends SubsystemBase{
    PivotIO io;
    PivotIOInputsAutoLogged inputs = new PivotIOInputsAutoLogged();
    RobotState robotState;

    LoggedTunableNumber position = new LoggedTunableNumber("Pivot/SetDegrees", 0.0);

    InterpolatingDoubleTreeMap treeMap = new InterpolatingDoubleTreeMap();

    public PivotSubsystem(PivotIO io, RobotState robotState) {
        this.io = io;
        this.robotState = robotState;

        treeMap.put(Inches.of(101.90).in(Meters), 0.0);
        treeMap.put(Inches.of(144.50).in(Meters), 3.0);
        treeMap.put(Inches.of(182.38).in(Meters), 5.0);
        treeMap.put(Inches.of(207.28).in(Meters), 9.0);
        treeMap.put(Inches.of(66.000).in(Meters), 0.0);
        treeMap.put(Inches.of(77.8).in(Meters), 0.0);
        treeMap.put(Inches.of(91.2).in(Meters), 0.0);
        treeMap.put(Inches.of(116.0).in(Meters), 1.0);
        treeMap.put(Inches.of(134.7).in(Meters), 3.0);
        treeMap.put(Inches.of(160.3).in(Meters), 4.0);

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
        Logger.processInputs("Pivot", inputs);

        Logger.recordOutput("Pivot/TreeMap Angle", treeMap.get(Double.valueOf(robotState.getDistanceFromHubMeters())));

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
            io.runSetpoint(Degrees.of(treeMap.get(Double.valueOf(robotState.getDistanceFromHubMeters()))))
        );
    
    }


}
