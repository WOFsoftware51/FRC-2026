package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.*;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase{
    IntakeIO io;
    IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

    public IntakeSubsystem(IntakeIO io) {
        this.io = io;
    }

    public Command runVolts(double volts) {
        return run(() -> 
            io.runVolts(Volts.of(volts))
        )
        .finallyDo(() ->
            io.stop()
        );
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Intake", inputs);
    }

}
