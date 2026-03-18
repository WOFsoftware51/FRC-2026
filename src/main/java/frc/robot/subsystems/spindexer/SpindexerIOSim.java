package frc.robot.subsystems.spindexer;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants;

public class SpindexerIOSim implements SpindexerIO {
    private TalonFX motorFront = new TalonFX(Constants.SpindexerConstants.kMotorFrontID, Constants.kCANIvoreName);
    private TalonFX motorBack = new TalonFX(Constants.SpindexerConstants.kMotorBackID, Constants.kCANIvoreName);

    private TalonFXSimState motorFrontSim = motorFront.getSimState();
    private TalonFXSimState motorBackSim = motorBack.getSimState();

    private DCMotorSim sim = new DCMotorSim(
        LinearSystemId.createDCMotorSystem(
            DCMotor.getKrakenX60(2),
            0.01, 
            Constants.SpindexerConstants.kGearRatio
        ), 
        DCMotor.getKrakenX60(2)
    );

    private TalonFXConfiguration configs = new TalonFXConfiguration();


    public SpindexerIOSim() {
        configs.MotorOutput.withInverted(InvertedValue.CounterClockwise_Positive);
        configs.MotorOutput.withNeutralMode(NeutralModeValue.Coast);
    

        motorFront.getConfigurator().apply(configs);

        motorBack.setControl(new Follower(Constants.SpindexerConstants.kMotorFrontID, MotorAlignmentValue.Aligned));
    }

    
    @Override
    public void updateInputs(SpindexerIOInputs inputs) {
        motorFrontSim.setSupplyVoltage(12.0);
        motorBackSim.setSupplyVoltage(12.0);
        double volts = motorFrontSim.getMotorVoltage();
        sim.setInputVoltage(volts);

        sim.update(0.02);
        double spindexerRotations = sim.getAngularPositionRotations();
        double spindexerRPM = sim.getAngularVelocityRPM();
        double spindexerRPMPerMinute = sim.getAngularAcceleration().in(RotationsPerSecondPerSecond)*60;
        
        double rotorRotations = spindexerRotations * Constants.SpindexerConstants.kGearRatio;
        double rotorRPS = (RPM.of(spindexerRPM).in(RotationsPerSecond)) * Constants.SpindexerConstants.kGearRatio;
        double rotorRPSPerSecond = ((spindexerRPMPerMinute)/60) * Constants.SpindexerConstants.kGearRatio;
        
        motorFrontSim.setRawRotorPosition(rotorRotations);
        motorFrontSim.setRotorVelocity(rotorRPS);
        motorFrontSim.setRotorAcceleration(rotorRPSPerSecond);
        motorBackSim.setRawRotorPosition(rotorRotations);
        motorBackSim.setRotorVelocity(rotorRPS);
        motorBackSim.setRotorAcceleration(rotorRPSPerSecond);


        inputs.velocity.mut_replace(spindexerRPM, RPM);
        inputs.acceleration.mut_replace(spindexerRPMPerMinute/60, RotationsPerSecondPerSecond);

        inputs.appliedVoltage.mut_replace(volts, Volts);

        inputs.supplyCurrent.mut_replace(sim.getCurrentDrawAmps(), Amps);
        inputs.torqueCurrent.mut_replace(sim.getCurrentDrawAmps(), Amps);
    }

    @Override
    public void runVolts(Voltage volts) {
        double clampedEffort = MathUtil.clamp(volts.magnitude(), -12, 12);
        motorFront.setVoltage(clampedEffort);
    }

    @Override
    public void stop() {
        runVolts(Volts.zero());
    }
    
    
}
