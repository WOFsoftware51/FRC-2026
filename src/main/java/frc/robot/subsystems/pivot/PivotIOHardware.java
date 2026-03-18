package frc.robot.subsystems.pivot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants;

public class PivotIOHardware implements PivotIO{
    private TalonFX motor = new TalonFX(Constants.PivotConstants.kMotorID, Constants.kCANIvoreName); //x44
    private CANcoder cancoder = new CANcoder(Constants.PivotConstants.kCANCoderID, Constants.kCANIvoreName);
    // 0 = -79.541016°
    // 27.17 = -262.265625°

    private TalonFXConfiguration configs = new TalonFXConfiguration();
    private CANcoderConfiguration canCoderConfigs = new CANcoderConfiguration();
  
    private double forwardLimit = (Constants.PivotConstants.kForwardLimit/360.0)*Constants.PivotConstants.kGearRatio;
    private double reverseLimit = (Constants.PivotConstants.kReverseLimit/360.0)*Constants.PivotConstants.kGearRatio;


    MotionMagicVoltage motion = new MotionMagicVoltage(0);
    double target = 0;


    public PivotIOHardware() {
        configs.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        configs.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        configs.SoftwareLimitSwitch.ForwardSoftLimitThreshold = forwardLimit;
        configs.SoftwareLimitSwitch.ReverseSoftLimitThreshold = reverseLimit;

        configs.MotionMagic.MotionMagicCruiseVelocity = 50;
        configs.MotionMagic.MotionMagicAcceleration = 600;

        configs.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        configs.Slot0.kP = 1.0;
        configs.Slot0.kI = 0.0;
        configs.Slot0.kD = 0.07999999821186066;
        configs.Slot0.kS = 0.0498046875;
        configs.Slot0.kV = 0.09099999815225601;
        configs.Slot0.kA = 0.0;
        configs.Slot0.kG = 0.0498046875;

        configs.ClosedLoopGeneral.ContinuousWrap = false;
        
        configs.MotorOutput.withInverted(InvertedValue.CounterClockwise_Positive);

        motor.getConfigurator().apply(configs);

        configs.CurrentLimits.StatorCurrentLimitEnable = true;
        configs.CurrentLimits.StatorCurrentLimit = 20;

        // cancoder.setPosition(0);

        canCoderConfigs.MagnetSensor.withSensorDirection(SensorDirectionValue.CounterClockwise_Positive);
        cancoder.getConfigurator().apply(canCoderConfigs);

        updateEncoder();
    }

    private double getCANCoderRotations() {
        double arm_CANcoder = cancoder.getAbsolutePosition().getValueAsDouble(); 
        return arm_CANcoder;
    }


    private void updateEncoder(){
        if(cancoder.isConnected()){
            motor.getConfigurator().setPosition(((getCANCoderRotations()-Constants.PivotConstants.kCANCoderOffset)/Constants.PivotConstants.kCANCoderGearRatio)*Constants.PivotConstants.kGearRatio);
        }
    }


    @Override
    public void updateInputs(PivotIOInputs inputs) {
        double motorRotations = motor.getPosition().getValueAsDouble();
        double motorRPS = motor.getVelocity().getValueAsDouble();
        double motorRPSPS = motor.getAcceleration().getValueAsDouble();

        double pivotDegrees = Rotations.of(motorRotations).in(Degrees)/Constants.PivotConstants.kGearRatio;
        double velocityDegreesPerSecond = RotationsPerSecond.of(motorRPS).in(DegreesPerSecond)/Constants.PivotConstants.kGearRatio;
        double velocityDegreesPerSecondPerSecond = RotationsPerSecondPerSecond.of(motorRPSPS).in(DegreesPerSecondPerSecond)/Constants.PivotConstants.kGearRatio;

        inputs.position.mut_replace(pivotDegrees, Degrees);
        inputs.targetPosition.mut_replace(target/360, Degrees);
        inputs.velocity.mut_replace(velocityDegreesPerSecond, DegreesPerSecond);
        inputs.acceleration.mut_replace(velocityDegreesPerSecondPerSecond, DegreesPerSecondPerSecond);

        inputs.appliedVoltage.mut_replace(motor.getMotorVoltage().getValueAsDouble(), Volts);

        inputs.supplyCurrent.mut_replace(motor.getSupplyCurrent().getValueAsDouble(), Amps);
        inputs.torqueCurrent.mut_replace(motor.getTorqueCurrent().getValueAsDouble(), Amps);

        inputs.canCoderPosition.mut_replace(cancoder.getAbsolutePosition().getValue());
    }

    @Override
    public void runVolts(Voltage volts) {
        double clampedEffort = MathUtil.clamp(volts.in(Volts), -12, 12);
        motor.setVoltage(clampedEffort);
    }

    @Override
    public void runSetpoint(Angle degrees) {
        this.target = (degrees.in(Rotations))*Constants.PivotConstants.kGearRatio;
        this.motion.withPosition(target);
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
