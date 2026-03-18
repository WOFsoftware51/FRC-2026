package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants;

public class IntakeIOHardware implements IntakeIO{
    private TalonFX motor = new TalonFX(Constants.IntakeConstants.kMotorID, Constants.kCANIvoreName);

    private TalonFXConfiguration configs = new TalonFXConfiguration();


    public IntakeIOHardware() {
        configs.MotorOutput.withInverted(InvertedValue.Clockwise_Positive);
        configs.MotorOutput.withNeutralMode(NeutralModeValue.Coast);

        configs.CurrentLimits.StatorCurrentLimitEnable = true;
        configs.CurrentLimits.StatorCurrentLimit = 60;

    
        motor.getConfigurator().apply(configs);
    }


    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        inputs.velocity.mut_replace(motor.getVelocity().getValueAsDouble(), RotationsPerSecond);
        inputs.acceleration.mut_replace(motor.getAcceleration().getValueAsDouble(), RotationsPerSecondPerSecond);

        inputs.appliedVoltage.mut_replace(motor.getMotorVoltage().getValueAsDouble(), Volts);

        inputs.supplyCurrent.mut_replace(motor.getSupplyCurrent().getValueAsDouble(), Amps);
        inputs.torqueCurrent.mut_replace(motor.getTorqueCurrent().getValueAsDouble(), Amps);
    }

    @Override
    public void runVolts(Voltage volts) {
        motor.setVoltage(volts.in(Volts));
    }

    @Override
    public void stop() {
        runVolts(Volts.zero());
    }
    
}
