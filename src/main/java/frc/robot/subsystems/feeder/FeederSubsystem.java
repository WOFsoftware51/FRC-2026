package frc.robot.subsystems.feeder;

import static edu.wpi.first.units.Units.*;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class FeederSubsystem extends SubsystemBase{
    FeederIO io;
    FeederIOInputsAutoLogged inputs = new FeederIOInputsAutoLogged();

    public FeederSubsystem(FeederIO io) {
        this.io = io;
    }

    @Override 
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Feeder", inputs);
    }

    public Command run() {
        return run(() ->
            io.runVolts(Volts.of(12))
        )
        .finallyDo(() ->
            io.stop()
        );
    }

    public Command runFeederVoltsCommand(double volts) {
        return run(() ->
            io.runVolts(Volts.of(volts))
        )
        .finallyDo(() ->
            io.stop()
        );
    }
}
