package frc.robot.subsystems.pivot;

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

public class PivotIOSim implements PivotIO {
    private TalonFX motor = new TalonFX(Constants.PivotConstants.kMotorID, Constants.kCANIvoreName);
    private TalonFXSimState motorSim = motor.getSimState();

    private DCMotorSim sim = new DCMotorSim(
        LinearSystemId.createDCMotorSystem(
            DCMotor.getKrakenX44(1), 
            0.01, 
            Constants.PivotConstants.kGearRatio
        ), 
        DCMotor.getKrakenX44(1)
    );

    private TalonFXConfiguration configs = new TalonFXConfiguration();
  
    private double forwardLimit = (Constants.PivotConstants.kForwardLimit/360.0)*Constants.PivotConstants.kGearRatio;
    private double reverseLimit = (Constants.PivotConstants.kReverseLimit/360.0)*Constants.PivotConstants.kGearRatio;


    MotionMagicVoltage motion = new MotionMagicVoltage(0);
    double target = 0;


    public PivotIOSim() {
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
    public void updateInputs(PivotIOInputs inputs) {
        motorSim.setSupplyVoltage(12);
        double volts = motorSim.getMotorVoltage();
        sim.setInputVoltage(volts);

        sim.update(0.02);
        double pivotDegrees = sim.getAngularPosition().in(Degree);
        double pivotDegPerSec = sim.getAngularVelocity().in(DegreesPerSecond);
        double pivotDegPerSecPerSec = sim.getAngularAcceleration().in(DegreesPerSecondPerSecond);

        double rotorRotations = Degrees.of(pivotDegrees).in(Rotations) * Constants.PivotConstants.kGearRatio;
        double rotorRotationsPerSecond = DegreesPerSecond.of(pivotDegPerSec).in(RotationsPerSecond) * Constants.PivotConstants.kGearRatio;
        double rotorRotationsPerSecondPerSecond = DegreesPerSecondPerSecond.of(pivotDegPerSec).in(RotationsPerSecondPerSecond) * Constants.PivotConstants.kGearRatio;

        motorSim.setRawRotorPosition(rotorRotations);
        motorSim.setRotorVelocity(rotorRotationsPerSecond);
        motorSim.setRotorAcceleration(rotorRotationsPerSecondPerSecond);

        inputs.position.mut_replace(pivotDegrees, Degrees);
        inputs.targetPosition.mut_replace(target/360, Degrees);
        
        inputs.velocity.mut_replace(pivotDegPerSec, DegreesPerSecond);
        inputs.acceleration.mut_replace(pivotDegPerSecPerSec, DegreesPerSecondPerSecond);

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
