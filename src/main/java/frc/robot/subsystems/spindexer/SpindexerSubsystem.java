package frc.robot.subsystems.spindexer;

import static edu.wpi.first.units.Units.*;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SpindexerSubsystem extends SubsystemBase {
  SpindexerIO io;
  SpindexerIOInputsAutoLogged inputs = new SpindexerIOInputsAutoLogged();

  public SpindexerSubsystem(SpindexerIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Spindexter", inputs);
  }

  public Command run() {
    return run(() -> 
      io.runVolts(Volts.of(12))
    )
    .finallyDo(() ->
      io.stop()
    );
  }

  public Command runSpindexerVoltsCommand(double volts) {
    return run(() -> 
      io.runVolts(Volts.of(volts))
    )
    .finallyDo(() ->
      io.stop()
    );
  }
}
