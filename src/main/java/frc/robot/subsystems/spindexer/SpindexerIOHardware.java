package frc.robot.subsystems.spindexer;

import static edu.wpi.first.units.Units.*;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants;

public class SpindexerIOHardware implements SpindexerIO {
    private TalonFX motorFront = new TalonFX(Constants.SpindexerConstants.kMotorFrontID, Constants.kCANIvoreName);
    private TalonFX motorBack = new TalonFX(Constants.SpindexerConstants.kMotorBackID, Constants.kCANIvoreName);

    private TalonFXConfiguration configs = new TalonFXConfiguration();
    private TalonFXConfiguration backConfigs = new TalonFXConfiguration();


    public SpindexerIOHardware() {
        configs.MotorOutput.withInverted(InvertedValue.Clockwise_Positive);
        configs.MotorOutput.withNeutralMode(NeutralModeValue.Brake);

        backConfigs.MotorOutput.withNeutralMode(NeutralModeValue.Brake);
        
        configs.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;
        configs.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;

        backConfigs.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;
        backConfigs.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;

        configs.CurrentLimits.StatorCurrentLimitEnable = true;
        configs.CurrentLimits.StatorCurrentLimit = 40;
        backConfigs.CurrentLimits.StatorCurrentLimitEnable = true;
        backConfigs.CurrentLimits.StatorCurrentLimit = 40;


        motorFront.getConfigurator().apply(configs);
        motorBack.getConfigurator().apply(backConfigs);

        motorBack.setControl(new Follower(Constants.SpindexerConstants.kMotorFrontID, MotorAlignmentValue.Aligned));
    }

    
    @Override
    public void updateInputs(SpindexerIOInputs inputs) {
        inputs.velocity.mut_replace(motorFront.getVelocity().getValueAsDouble(), RotationsPerSecond);
        inputs.acceleration.mut_replace(motorFront.getAcceleration().getValueAsDouble(), RotationsPerSecondPerSecond);

        inputs.appliedVoltage.mut_replace(motorFront.getMotorVoltage().getValueAsDouble(), Volts);

        inputs.supplyCurrent.mut_replace(motorFront.getSupplyCurrent().getValueAsDouble(), Amps);
        inputs.torqueCurrent.mut_replace(motorFront.getTorqueCurrent().getValueAsDouble(), Amps);

        inputs.velocityBACK.mut_replace(motorBack.getVelocity().getValue());
        inputs.appliedVoltageBACK.mut_replace(motorBack.getMotorVoltage().getValue());

    }

    @Override
    public void runVolts(Voltage volts) {
        double clampedEffort = MathUtil.clamp(volts.magnitude(), -12, 12);
        motorFront.setControl(new VoltageOut(clampedEffort).withEnableFOC(true));
    }

    @Override
    public void stop() {
        runVolts(Volts.zero());
    }
    
}
