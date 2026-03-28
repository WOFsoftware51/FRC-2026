package frc.robot.subsystems.hood;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants;

public class HoodIOSim implements HoodIO {
    private TalonFX motor = new TalonFX(Constants.HoodConstants.kMotorID, Constants.kCANIvoreName);
    private TalonFXSimState motorSim = motor.getSimState();

    private DCMotorSim sim = new DCMotorSim(
        LinearSystemId.createDCMotorSystem(
            DCMotor.getKrakenX44(1), 
            0.01, 
            Constants.HoodConstants.kGearRatio
        ), 
        DCMotor.getKrakenX44(1)
    );

    private TalonFXConfiguration configs = new TalonFXConfiguration();
  
    private double forwardLimit = (Constants.HoodConstants.kForwardLimit/360.0)*Constants.HoodConstants.kGearRatio;
    private double reverseLimit = (Constants.HoodConstants.kReverseLimit/360.0)*Constants.HoodConstants.kGearRatio;


    MotionMagicVoltage motion = new MotionMagicVoltage(0);
    double target = 0;


    public HoodIOSim() {
        configs.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;
        configs.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;
        configs.SoftwareLimitSwitch.ForwardSoftLimitThreshold = forwardLimit;
        configs.SoftwareLimitSwitch.ReverseSoftLimitThreshold = reverseLimit;

        configs.MotionMagic.MotionMagicCruiseVelocity = 0;
        configs.MotionMagic.MotionMagicAcceleration = 0;

        configs.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        configs.Slot0.kP = 0.0;
        configs.Slot0.kI = 0.0;
        configs.Slot0.kD = 0.0;
        configs.Slot0.kV = 0.0;
        configs.Slot0.kA = 0.0;
        configs.Slot0.kS = 0.0;

        configs.ClosedLoopGeneral.ContinuousWrap = false;
        
        configs.MotorOutput.withInverted(InvertedValue.CounterClockwise_Positive);

        motor.getConfigurator().apply(configs);

    }

    @Override
    public void updateInputs(HoodIOInputs inputs) {
        motorSim.setSupplyVoltage(12);
        double volts = motorSim.getMotorVoltage();
        sim.setInputVoltage(volts);

        sim.update(0.02);
        double hoodDegrees = sim.getAngularPosition().in(Degree);
        double hoodDegPerSec = sim.getAngularVelocity().in(DegreesPerSecond);
        double hoodDegPerSecPerSec = sim.getAngularAcceleration().in(DegreesPerSecondPerSecond);

        double rotorRotations = Degrees.of(hoodDegrees).in(Rotations) * Constants.HoodConstants.kGearRatio;
        double rotorRotationsPerSecond = DegreesPerSecond.of(hoodDegPerSec).in(RotationsPerSecond) * Constants.HoodConstants.kGearRatio;
        double rotorRotationsPerSecondPerSecond = DegreesPerSecondPerSecond.of(hoodDegPerSec).in(RotationsPerSecondPerSecond) * Constants.HoodConstants.kGearRatio;

        motorSim.setRawRotorPosition(rotorRotations);
        motorSim.setRotorVelocity(rotorRotationsPerSecond);
        motorSim.setRotorAcceleration(rotorRotationsPerSecondPerSecond);

        inputs.position.mut_replace(hoodDegrees, Degrees);
        
        inputs.velocity.mut_replace(hoodDegPerSec, DegreesPerSecond);
        inputs.acceleration.mut_replace(hoodDegPerSecPerSec, DegreesPerSecondPerSecond);

        inputs.appliedVoltage.mut_replace(volts, Volts);

        inputs.supplyCurrent.mut_replace(sim.getCurrentDrawAmps(), Amps);
        inputs.torqueCurrent.mut_replace(sim.getCurrentDrawAmps(), Amps);
    }

    @Override
    public void runVolts(Voltage volts) {
        double clampedEffort = MathUtil.clamp(volts.in(Volts), -12, 12);
        motor.setVoltage(clampedEffort);
    }

    @Override
    public void runSetpoint(Angle degrees) {
        this.target = (degrees.in(Rotations))*Constants.TurretConstants.kGearRatio;
        this.motion.Position = target;
        motor.setControl(motion);
    }

    @Override
    public void resetEncoder() {
        motor.setPosition(0);
    }

    @Override
    public void stop() {
        runVolts(Volts.zero());
    }
 
}
