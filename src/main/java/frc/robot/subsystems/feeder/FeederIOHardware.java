package frc.robot.subsystems.feeder;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants;

public class FeederIOHardware implements FeederIO {
    private TalonFX motor = new TalonFX(Constants.FeederConstants.kMotorID, Constants.kCANIvoreName);

    private TalonFXConfiguration configs = new TalonFXConfiguration();

    

    public FeederIOHardware() {
        configs.MotorOutput.withInverted(InvertedValue.Clockwise_Positive);
        configs.MotorOutput.withNeutralMode(NeutralModeValue.Coast);
    
        configs.CurrentLimits.StatorCurrentLimitEnable = true;
        configs.CurrentLimits.StatorCurrentLimit = 80;

        motor.getConfigurator().apply(configs);
    }


    @Override
    public void updateInputs(FeederIOInputs inputs) {
        inputs.velocity.mut_replace(motor.getVelocity().getValueAsDouble(), RotationsPerSecond);
        inputs.acceleration.mut_replace(motor.getAcceleration().getValueAsDouble(), RotationsPerSecondPerSecond);

        inputs.appliedVoltage.mut_replace(motor.getMotorVoltage().getValueAsDouble(), Volts);

        inputs.supplyCurrent.mut_replace(motor.getSupplyCurrent().getValueAsDouble(), Amps);
        inputs.torqueCurrent.mut_replace(motor.getTorqueCurrent().getValueAsDouble(), Amps);
    }

    @Override
    public void runVolts(Voltage volts) {
        double clampedEffort = MathUtil.clamp(volts.magnitude(), -12, 12);
        motor.setVoltage(clampedEffort);
    }

    @Override
    public void stop() {
        runVolts(Volts.zero());
    }
}
