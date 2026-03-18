package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.*;

import org.ironmaple.simulation.IntakeSimulation;
import org.ironmaple.simulation.drivesims.AbstractDriveTrainSimulation;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.Constants;

public class IntakeIOSim implements IntakeIO{
    private TalonFX motor = new TalonFX(Constants.IntakeConstants.kMotorID, Constants.kCANIvoreName);
    private TalonFXSimState motorSim = motor.getSimState();

    private DCMotorSim sim = new DCMotorSim(
        LinearSystemId.createDCMotorSystem(
            DCMotor.getKrakenX60(1), 
            0.01, 
            Constants.IntakeConstants.kGearRatio
        ),
        DCMotor.getKrakenX60(1)
    );

    
    private TalonFXConfiguration configs = new TalonFXConfiguration();


    public IntakeIOSim(AbstractDriveTrainSimulation mapleSim) {
        configs.MotorOutput.withInverted(InvertedValue.Clockwise_Positive);
        configs.MotorOutput.withNeutralMode(NeutralModeValue.Coast);

        
    
        motor.getConfigurator().apply(configs);
    }


    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        motorSim.setSupplyVoltage(12.0);
        double volts = motorSim.getMotorVoltage();
        sim.setInputVoltage(volts);

        sim.update(0.02);
        double intakeRotations = sim.getAngularPositionRotations();
        double intakeRPM = sim.getAngularVelocityRPM();
        double intakeRPMPerMinute = sim.getAngularAcceleration().in(RotationsPerSecondPerSecond)*60;
        
        double rotorRotations = intakeRotations * Constants.IntakeConstants.kGearRatio;
        double rotorRPS = (RPM.of(intakeRPM).in(RotationsPerSecond)) * Constants.IntakeConstants.kGearRatio;
        double rotorRPSPerSecond = ((intakeRPMPerMinute)/60) * Constants.IntakeConstants.kGearRatio;
        
        motorSim.setRawRotorPosition(rotorRotations);
        motorSim.setRotorVelocity(rotorRPS);
        motorSim.setRotorAcceleration(rotorRPSPerSecond);


        inputs.velocity.mut_replace(intakeRPM, RPM);
        inputs.acceleration.mut_replace(intakeRPMPerMinute/60, RotationsPerSecondPerSecond);

        inputs.appliedVoltage.mut_replace(volts, Volts);

        inputs.supplyCurrent.mut_replace(sim.getCurrentDrawAmps(), Amps);
        inputs.torqueCurrent.mut_replace(sim.getCurrentDrawAmps(), Amps);
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
