package frc.robot.subsystems.turret;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants;

public class TurretIOHardware implements TurretIO {
  private TalonFX motor = new TalonFX(Constants.TurretConstants.kMotorID, Constants.kCANIvoreName); //x44
  private TalonFXConfiguration configs = new TalonFXConfiguration();
  
  private double forwardLimit = (Constants.TurretConstants.kForwardLimit/360.0)*Constants.TurretConstants.kGearRatio;
  private double reverseLimit = (Constants.TurretConstants.kReverseLimit/360.0)*Constants.TurretConstants.kGearRatio;
  

  MotionMagicVoltage motion = new MotionMagicVoltage(0);


  public TurretIOHardware() {
    configs.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;
    configs.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;
    configs.SoftwareLimitSwitch.ForwardSoftLimitThreshold = forwardLimit;
    configs.SoftwareLimitSwitch.ReverseSoftLimitThreshold = reverseLimit;

    configs.MotionMagic.MotionMagicCruiseVelocity = 90;
    configs.MotionMagic.MotionMagicAcceleration = 400;

    configs.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    configs.Slot0.kP = 1.0;
    configs.Slot0.kI = 0.0;
    configs.Slot0.kD = 0.12;
    configs.Slot0.kV = 0.095;
    configs.Slot0.kA = 0.0;
    configs.Slot0.kS = 0.022;

    configs.ClosedLoopGeneral.ContinuousWrap = false;
    
    configs.MotorOutput.withInverted(InvertedValue.CounterClockwise_Positive);

    motor.getConfigurator().apply(configs);
    
  }

  
  @Override
  public void runVolts(Voltage volts) {
    double clampedEffort = MathUtil.clamp(volts.in(Volts), -12, 12);
    motor.setVoltage(clampedEffort);
  }
  
  @Override
  public void runSetpoint(Angle degrees) {
    double target = (degrees.in(Rotations))*Constants.TurretConstants.kGearRatio;
    this.motion.Position = target;
    motor.setControl(motion);
  }
  
  
  @Override
  public void stop() {
    runVolts(Volts.zero());
  }
  
  
  @Override
  public void resetEncoder() {
    motor.setPosition(0);
  }
  
  
  @Override
  public void updateInputs(TurretIOInputs inputs) {
    double motorRotations = motor.getPosition().getValueAsDouble();
    double motorRPS = motor.getVelocity().getValueAsDouble();
    double motorRPSPS = motor.getAcceleration().getValueAsDouble();

    double turretDegrees = Rotations.of(motorRotations).in(Degrees)/Constants.TurretConstants.kGearRatio;
    double velocityDegreesPerSecond = RotationsPerSecond.of(motorRPS).in(DegreesPerSecond)/Constants.TurretConstants.kGearRatio;
    double velocityDegreesPerSecondPerSecond = RotationsPerSecondPerSecond.of(motorRPSPS).in(DegreesPerSecondPerSecond)/Constants.TurretConstants.kGearRatio;

    inputs.position.mut_replace(turretDegrees, Degrees);
    inputs.velocity.mut_replace(velocityDegreesPerSecond, DegreesPerSecond);
    inputs.acceleration.mut_replace(velocityDegreesPerSecondPerSecond, DegreesPerSecondPerSecond);

    inputs.appliedVoltage.mut_replace(motor.getMotorVoltage().getValueAsDouble(), Volts);

    inputs.supplyCurrent.mut_replace(motor.getSupplyCurrent().getValueAsDouble(), Amps);
    inputs.torqueCurrent.mut_replace(motor.getTorqueCurrent().getValueAsDouble(), Amps);

  }

    
}
