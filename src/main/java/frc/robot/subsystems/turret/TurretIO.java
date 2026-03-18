package frc.robot.subsystems.turret;

import static edu.wpi.first.units.Units.*;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularAcceleration;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutCurrent;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Voltage;


public interface TurretIO {
    @AutoLog
    class TurretIOInputs {
        public MutAngle position = Degrees.mutable(0);
        public MutAngle targetPosition = Degrees.mutable(0);

        public MutAngularVelocity velocity = DegreesPerSecond.mutable(0);
        public MutAngularAcceleration acceleration = DegreesPerSecondPerSecond.mutable(0);

        public MutVoltage appliedVoltage = Volts.mutable(0);

        public MutCurrent supplyCurrent = Amps.mutable(0);
        public MutCurrent torqueCurrent = Amps.mutable(0);
    }

    void updateInputs(TurretIOInputs inputs);
    void runVolts(Voltage volts);
    void runSetpoint(Angle degrees);
    void resetEncoder();
    void stop();
}
