package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.*;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.measure.MutAngularAcceleration;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutCurrent;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Voltage;

public interface IntakeIO {
    @AutoLog
    class IntakeIOInputs {
        public MutAngularVelocity velocity = DegreesPerSecond.mutable(0);
        public MutAngularAcceleration acceleration = DegreesPerSecondPerSecond.mutable(0);

        public MutVoltage appliedVoltage = Volts.mutable(0);

        public MutCurrent supplyCurrent = Amps.mutable(0);
        public MutCurrent torqueCurrent = Amps.mutable(0);
    }
    void updateInputs(IntakeIOInputs inputs);
    void runVolts(Voltage volts);
    void stop();

}
