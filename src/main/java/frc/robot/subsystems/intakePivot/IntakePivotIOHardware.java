package frc.robot.subsystems.intakePivot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants;

public class IntakePivotIOHardware implements IntakePivotIO{
    private TalonFX motor = new TalonFX(Constants.IntakePivotConstants.kMotorID, Constants.kCANIvoreName);
    private CANcoder cancoder = new CANcoder(Constants.IntakePivotConstants.kCANcoderID, Constants.kCANIvoreName);
    private DigitalInput limitSwitch = new DigitalInput(5);

    private TalonFXConfiguration configs = new TalonFXConfiguration();
    private CANcoderConfiguration canCoderConfigs = new CANcoderConfiguration();

    private double forwardLimit = (Constants.IntakePivotConstants.kForwardLimit/360.0)*Constants.IntakePivotConstants.kGearRatio;
    private double reverseLimit = (Constants.IntakePivotConstants.kReverseLimit/360.0)*Constants.IntakePivotConstants.kGearRatio;

    MotionMagicVoltage motion = new MotionMagicVoltage(0);
    double target = 0;


    public IntakePivotIOHardware() {
        configs.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;
        configs.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;
        configs.SoftwareLimitSwitch.ForwardSoftLimitThreshold = forwardLimit;
        configs.SoftwareLimitSwitch.ReverseSoftLimitThreshold = reverseLimit;

        configs.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        configs.MotionMagic.MotionMagicCruiseVelocity = 30;
        configs.MotionMagic.MotionMagicAcceleration = 30;

        configs.Slot0.kP = 0.0;
        configs.Slot0.kI = 0.0;
        configs.Slot0.kD = 0.0;
        configs.Slot0.kS = 0.0;
        configs.Slot0.kV = 0.112;
        configs.Slot0.kA = 0.0;

        configs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        configs.CurrentLimits.StatorCurrentLimitEnable = true;
        configs.CurrentLimits.StatorCurrentLimit = 40;


        motor.getConfigurator().apply(configs);

        canCoderConfigs.MagnetSensor.withSensorDirection(SensorDirectionValue.CounterClockwise_Positive);
        cancoder.getConfigurator().apply(canCoderConfigs);

        // cancoder.setPosition(0);

        updateEncoder();
    }


    @Override
    public void updateInputs(IntakePivotIOInputs inputs) {
        double motorRotations = motor.getPosition().getValueAsDouble();
        double motorRPS = motor.getVelocity().getValueAsDouble();
        double motorRPSPS = motor.getAcceleration().getValueAsDouble();

        double pivotDegrees = Rotations.of(motorRotations).in(Degrees)/Constants.IntakePivotConstants.kGearRatio;
        double velocityDegreesPerSecond = RotationsPerSecond.of(motorRPS).in(DegreesPerSecond)/Constants.IntakePivotConstants.kGearRatio;
        double velocityDegreesPerSecondPerSecond = RotationsPerSecondPerSecond.of(motorRPSPS).in(DegreesPerSecondPerSecond)/Constants.IntakePivotConstants.kGearRatio;

        inputs.position.mut_replace(pivotDegrees, Degrees);

        inputs.velocity.mut_replace(velocityDegreesPerSecond, DegreesPerSecond);
        inputs.acceleration.mut_replace(velocityDegreesPerSecondPerSecond, DegreesPerSecondPerSecond);

        inputs.appliedVoltage.mut_replace(motor.getMotorVoltage().getValue());

        inputs.supplyCurrent.mut_replace(motor.getSupplyCurrent().getValue());
        inputs.torqueCurrent.mut_replace(motor.getTorqueCurrent().getValue());

        inputs.canCoderPosition.mut_replace(cancoder.getPosition().getValue());

        inputs.limitSwitchBoolean = getLimitSwitch();
    }

    private boolean getLimitSwitch() {
        return !limitSwitch.get();
    }

    @Override
    public void runVolts(Voltage volts) {
        double clampedEffort = MathUtil.clamp(volts.in(Volts), -12, 12);
        motor.setControl(new VoltageOut(clampedEffort).withEnableFOC(true).withLimitForwardMotion(getLimitSwitch()));
    }


    @Override
    public void runSetpoint(Angle degrees) {
        this.target = degrees.in(Degrees);
        this.motion.withPosition(target).withEnableFOC(true).withLimitForwardMotion(getLimitSwitch());
        motor.setControl(this.motion);
    }


    @Override
    public void resetEncoder() {
        motor.setPosition(0);
    }


    @Override
    public void stop() {
        runVolts(Volts.zero());
    }

    private double getCANCoderRotations() {
        double arm_CANcoder = cancoder.getAbsolutePosition().getValueAsDouble(); 
        return arm_CANcoder;
    }

    private void updateEncoder(){
        if(cancoder.isConnected()){
            motor.getConfigurator().setPosition(((getCANCoderRotations()-Constants.IntakePivotConstants.kCANCoderOffset)/Constants.IntakePivotConstants.kCANCoderGearRatio)*Constants.IntakePivotConstants.kGearRatio);
        }
    }

}
