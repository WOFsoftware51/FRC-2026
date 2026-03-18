package frc.robot.subsystems.feeder;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants;

public class FeederIOSim implements FeederIO {
    private TalonFX motor = new TalonFX(Constants.FeederConstants.kMotorID, Constants.kCANIvoreName);
    private TalonFXSimState motorSim = motor.getSimState();
    private DCMotorSim sim = new DCMotorSim(
        LinearSystemId.createDCMotorSystem(
            DCMotor.getKrakenX60(1), 
            0.01, 
            Constants.FeederConstants.kGearRatio
        ),
        DCMotor.getKrakenX60(1)
    );

    private TalonFXConfiguration configs = new TalonFXConfiguration();
    

    public FeederIOSim() {
        configs.MotorOutput.withInverted(InvertedValue.Clockwise_Positive);
        configs.MotorOutput.withNeutralMode(NeutralModeValue.Coast);
    
        motor.getConfigurator().apply(configs);
    }


    @Override
    public void updateInputs(FeederIOInputs inputs) {
        motorSim.setSupplyVoltage(12.0);
        double volts = motorSim.getMotorVoltage();
        sim.setInputVoltage(volts);

        sim.update(0.02);
        double feederRotations = sim.getAngularPositionRotations();
        double feederRPM = sim.getAngularVelocityRPM();
        double feederRPMPerMinute = sim.getAngularAcceleration().in(RotationsPerSecondPerSecond)*60;
        
        double rotorRotations = feederRotations * Constants.FeederConstants.kGearRatio;
        double rotorRPS = (RPM.of(feederRPM).in(RotationsPerSecond)) * Constants.FeederConstants.kGearRatio;
        double rotorRPSPerSecond = ((feederRPMPerMinute)/60) * Constants.FeederConstants.kGearRatio;
        
        motorSim.setRawRotorPosition(rotorRotations);
        motorSim.setRotorVelocity(rotorRPS);
        motorSim.setRotorAcceleration(rotorRPSPerSecond);


        inputs.velocity.mut_replace(feederRPM, RPM);
        inputs.acceleration.mut_replace(feederRPMPerMinute/60, RotationsPerSecondPerSecond);

        inputs.appliedVoltage.mut_replace(volts, Volts);

        inputs.supplyCurrent.mut_replace(sim.getCurrentDrawAmps(), Amps);
        inputs.torqueCurrent.mut_replace(sim.getCurrentDrawAmps(), Amps);
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
