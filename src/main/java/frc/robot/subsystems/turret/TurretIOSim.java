package frc.robot.subsystems.turret;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
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

public class TurretIOSim implements TurretIO {
    private final TalonFX motor = new TalonFX(Constants.TurretConstants.kMotorID);
    private final TalonFXSimState motorSim = motor.getSimState();
    private final DCMotorSim sim = new DCMotorSim(
        LinearSystemId.createDCMotorSystem(
            DCMotor.getKrakenX44(1), 
            0.001,
            Constants.TurretConstants.kGearRatio
        ),
        DCMotor.getKrakenX44(1)
    );
    
    private double target = 0;

    private double forwardLimit = (Constants.TurretConstants.kForwardLimit/360.0)*Constants.TurretConstants.kGearRatio;
    private double reverseLimit = (Constants.TurretConstants.kReverseLimit/360.0)*Constants.TurretConstants.kGearRatio;
    private TalonFXConfiguration configs = new TalonFXConfiguration();

    MotionMagicVoltage motion = new MotionMagicVoltage(0);

    public TurretIOSim() {
    configs.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    configs.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    configs.SoftwareLimitSwitch.ForwardSoftLimitThreshold = forwardLimit;
    configs.SoftwareLimitSwitch.ReverseSoftLimitThreshold = reverseLimit;

    configs.MotionMagic.MotionMagicCruiseVelocity = 60;
    configs.MotionMagic.MotionMagicAcceleration = 300;

    configs.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    configs.Slot0.kP = 1.0;
    configs.Slot0.kI = 0.0;
    configs.Slot0.kD = 0.05;
    configs.Slot0.kS = 0.9;
    configs.Slot0.kV = 0.094;
    configs.Slot0.kA = 0.0;

    configs.ClosedLoopGeneral.ContinuousWrap = false;
    
    configs.MotorOutput.withInverted(InvertedValue.CounterClockwise_Positive);

    configs.CurrentLimits.StatorCurrentLimit = 35;
    configs.CurrentLimits.StatorCurrentLimitEnable = false;
    configs.CurrentLimits.SupplyCurrentLimit = 10;
    configs.CurrentLimits.SupplyCurrentLimitEnable = false;

    motor.getConfigurator().apply(configs);

    // resetEncoder();
    
    }


    @Override
    public void updateInputs(TurretIOInputs inputs) {
        motorSim.setSupplyVoltage(12.0);
        double volts = motorSim.getMotorVoltage();
        sim.setInputVoltage(volts);

        sim.update(0.02);
        double turretRotations = sim.getAngularPositionRotations();
        double turretRPM = sim.getAngularVelocityRPM();
        double turretRPMPerMinute = sim.getAngularAcceleration().in(DegreesPerSecondPerSecond)*60;
        
        double rotorRotations = turretRotations * Constants.TurretConstants.kGearRatio;
        double rotorRPS = RPM.of(turretRPM).in(RotationsPerSecond) * Constants.TurretConstants.kGearRatio;
        double rotorRPSPerSecond = (turretRPMPerMinute)/60 * Constants.FeederConstants.kGearRatio;
        
        motorSim.setRawRotorPosition(rotorRotations);
        motorSim.setRotorVelocity(rotorRPS);
        motorSim.setRotorAcceleration(rotorRPSPerSecond);
        
        double positionDegrees = Rotations.of(turretRotations).in(Degrees);
        double velocityDegreesPerSecond = RPM.of(turretRPM).in(DegreesPerSecond);
        double velocityDegreesPerSecondPerSecond = turretRPMPerMinute/60;
        
        inputs.position.mut_replace(positionDegrees, Degrees);
        inputs.targetPosition.mut_replace(target, Degrees);

        inputs.velocity.mut_replace(velocityDegreesPerSecond, DegreesPerSecond);
        inputs.acceleration.mut_replace(velocityDegreesPerSecondPerSecond, DegreesPerSecondPerSecond);

        
        inputs.appliedVoltage.mut_replace(volts, Volts);
        
        inputs.supplyCurrent.mut_replace(sim.getCurrentDrawAmps(), Amps);
        inputs.torqueCurrent.mut_replace(sim.getCurrentDrawAmps(), Amps);
        
    }

  
  @Override
  public void runVolts(Voltage volts) {
    double clampedEffort = MathUtil.clamp(volts.in(Volts), -12, 12);
    motor.setControl(new VoltageOut(clampedEffort).withEnableFOC(true));
  }
  
  @Override
  public void runSetpoint(Angle degrees) {
    double target = (degrees.in(Rotations))*Constants.TurretConstants.kGearRatio;
    this.motion.withPosition(target).withEnableFOC(true).withFeedForward(0);
    motor.setControl(motion);
  }

  private double springFix() {
    double constant = 8;
    return 1;
  }
  
  
  @Override
  public void stop() {
    runVolts(Volts.zero());
  }
  
  
  @Override
  public void resetEncoder() {
    motor.setPosition(0);
  }
  
     
}
