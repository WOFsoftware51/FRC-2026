// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.LoggedTunableNumber;

public class IntakePivotSubsystem extends SubsystemBase {
  /** Creates a new IntakePivotSubsystem. */
  private TalonFX motor = new TalonFX(Constants.IntakePivotConstants.kMotorID, Constants.kCANIvoreName);
  private CANcoder cancoder = new CANcoder(Constants.IntakePivotConstants.kCANcoderID, Constants.kCANIvoreName);

  private TalonFXConfiguration configs = new TalonFXConfiguration();
  private CANcoderConfiguration canCoderConfigs = new CANcoderConfiguration();

  private double forwardLimit = (Constants.IntakePivotConstants.kForwardLimit/360.0)*Constants.IntakePivotConstants.kGearRatio;
  private double reverseLimit = (Constants.IntakePivotConstants.kReverseLimit/360.0)*Constants.IntakePivotConstants.kGearRatio;

  MotionMagicVoltage motion = new MotionMagicVoltage(0);

  LoggedTunableNumber timeOut = new LoggedTunableNumber("IntakePivot/timout", 0);


  public IntakePivotSubsystem() {
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

  public Command runVoltsJoystick(DoubleSupplier volts) {
    return run(() -> 
      motor.setVoltage(volts.getAsDouble()*12)
    )
    .finallyDo(() ->
      stop()
    );
  }


  public Command runVolts(double volts) {
    return run(() -> 
      motor.setVoltage(volts)
    )
    .finallyDo(() ->
      stop()
    );
  }

  public double getRotationsUntilSetpoint(Angle rotations) {
    double motorRotations = motor.getPosition().getValueAsDouble()/Constants.IntakePivotConstants.kGearRatio;

    double value = Math.abs(motorRotations - rotations.in(Rotations));

    return value;
  }

  // public Command runVoltsToSetpoint(Angle setpoint) {
  //   return run(
  //     runVolts(6*getRotationsUntilSetpoint(setpoint))
  //   );
    
    
  // }

  public void runCarefully() {
    double volts = ((6 * Math.abs(getCANCoderDegrees()/5) + 0.5));
    double clampedEffort = MathUtil.clamp(volts, -12, 12);

    Logger.recordOutput("IntakePivot/runCarefully Volts", volts);
    
    if(!(-getCANCoderDegrees()<5)) {
      motor.setVoltage(clampedEffort);
    }
    else{
      motor.setVoltage(0);
    }
  }


  public Command runPivot2Volts() {
    return Commands.run(()->
      motor.setVoltage(2)
    );
  }

  public void runPivotTimeBased() {
    double seconds = 0;

    if(seconds > timeOut.get()) {
      stop();
    }
    else{
      motor.setVoltage(2);
      seconds++;
    }
  }

    public Command runPivotTimeBasedCommand() {
      return Commands.run(() ->
        runPivotTimeBasedCommand()
      );
    }


  public void runSetpoint(Angle degrees) {
    double target = (degrees.in(Rotations))*Constants.IntakePivotConstants.kGearRatio;
    this.motion.Position = target;
    motor.setControl(motion);
  }

  public Command runSetpointCommand(double degrees) {
    return run(() ->
      runSetpoint(Degrees.of(degrees))
    )
    .finallyDo(() -> 
      stop()
    );
  } 

  public Command resetEncoder() {
    return runOnce(() ->
      motor.setPosition(0)
    );
  }

  public double getCANCoderRotations() {
    double arm_CANcoder = cancoder.getAbsolutePosition().getValueAsDouble(); 
    return arm_CANcoder;
  }

  public double getCANCoderDegrees() {
    double arm_CANcoder = (cancoder.getAbsolutePosition().getValueAsDouble()*360)/Constants.IntakePivotConstants.kCANCoderGearRatio;
    return arm_CANcoder;
  }


  private void updateEncoder(){
      if(cancoder.isConnected()){
          motor.getConfigurator().setPosition(((getCANCoderRotations()-Constants.IntakePivotConstants.kCANCoderOffset)/Constants.IntakePivotConstants.kCANCoderGearRatio)*Constants.IntakePivotConstants.kGearRatio);
      }
  }


  public void stop() {
    motor.setVoltage(0);
  }

  public Command stopCommand() {
    return Commands.runOnce(()->
      motor.setVoltage(0)
    );
  }


  @Override
  public void periodic() {
    Logger.recordOutput("IntakePivot/Position", motor.getPosition().getValueAsDouble()*360/Constants.IntakePivotConstants.kGearRatio);
    Logger.recordOutput("IntakePivot/Volts", motor.getMotorVoltage().getValueAsDouble());
    Logger.recordOutput("IntakePivot/Velocity", motor.getVelocity().getValueAsDouble()*360/Constants.IntakePivotConstants.kGearRatio);

    Logger.recordOutput("IntakePivot/Torque", motor.getTorqueCurrent().getValueAsDouble());

    Logger.recordOutput("IntakePivot/CANCoder Position", cancoder.getAbsolutePosition().getValueAsDouble());
    Logger.recordOutput("IntakePivot/CANCoder Degrees", (cancoder.getAbsolutePosition().getValueAsDouble()*360)/Constants.IntakePivotConstants.kCANCoderGearRatio);
  }
}
