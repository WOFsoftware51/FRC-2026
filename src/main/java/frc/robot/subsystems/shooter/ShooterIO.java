package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.*;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.MutAngularAcceleration;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutCurrent;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Voltage;


public interface ShooterIO {
    @AutoLog
    class ShooterIOInputs {
        public MutAngularVelocity currentVelocity = RPM.mutable(0);
        public MutAngularVelocity targetVelocity = RPM.mutable(0);
        
        public MutAngularAcceleration currentAcceleration = DegreesPerSecondPerSecond.mutable(0);

        public MutVoltage appliedVoltage = Volts.mutable(0);

        public MutCurrent supplyCurrent = Amps.mutable(0);
        public MutCurrent torqueCurrent = Amps.mutable(0);

    }
    void updateInputs(ShooterIOInputs inputs);
    void runVolts(Voltage volts);
    void runVelocityRPM(AngularVelocity velocityRPM);
    void updateGains(double... gains);
    void stop();
}
