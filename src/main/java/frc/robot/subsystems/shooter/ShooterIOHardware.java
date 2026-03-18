package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants;

public class ShooterIOHardware implements ShooterIO{
    private TalonFX motorLeft = new TalonFX(Constants.ShooterConstants.kMotorLeftID, Constants.kCANIvoreName);
    private TalonFX motorRight = new TalonFX(Constants.ShooterConstants.kMotorRightID, Constants.kCANIvoreName);

    private TalonFXConfiguration configs = new TalonFXConfiguration();

    Slot0Configs testSlots = new Slot0Configs();

    private VelocityVoltage speed = new VelocityVoltage(0);
    private double targetVelocity = 0;



    public ShooterIOHardware() {    
        configs.MotorOutput.withInverted(InvertedValue.CounterClockwise_Positive);

        configs.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        configs.Slot0.kP = 0.7;
        configs.Slot0.kI = 0.0;
        configs.Slot0.kD = 0.001;
        configs.Slot0.kV = 0.12909;
        configs.Slot0.kA = 1.0;
        configs.Slot0.kS = 0.0;

        configs.CurrentLimits.StatorCurrentLimitEnable = true;
        configs.CurrentLimits.StatorCurrentLimit = 180;

        motorLeft.getConfigurator().apply(configs);

        motorRight.setControl(new Follower(Constants.ShooterConstants.kMotorLeftID, MotorAlignmentValue.Opposed));
    }

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        inputs.currentVelocity.mut_replace(motorLeft.getVelocity().getValueAsDouble(), RotationsPerSecond);
        inputs.targetVelocity.mut_replace(targetVelocity, RotationsPerSecond);
        inputs.currentAcceleration.mut_replace(motorLeft.getAcceleration().getValueAsDouble(), RotationsPerSecondPerSecond);

        inputs.appliedVoltage.mut_replace(motorLeft.getMotorVoltage().getValueAsDouble(), Volts);

        inputs.supplyCurrent.mut_replace(motorLeft.getSupplyCurrent().getValueAsDouble(), Amps);
        inputs.torqueCurrent.mut_replace(motorLeft.getTorqueCurrent().getValueAsDouble(), Amps);
    }

    @Override
    public void runVolts(Voltage volts) {
        double clampedEffort = MathUtil.clamp(volts.magnitude(), -12, 12);
        motorLeft.setVoltage(clampedEffort);
    }

    @Override
    public void runVelocityRPM(AngularVelocity velocityRPM) {
        this.targetVelocity = (velocityRPM.in(RotationsPerSecond))*Constants.ShooterConstants.kGearRatio;
        this.speed.withVelocity(this.targetVelocity);
        motorLeft.setControl(speed);
    }

    @Override
    public void updateGains(double... gains) {
        testSlots.withKP(gains[0]);
        testSlots.withKI(gains[1]);
        testSlots.withKD(gains[2]);
        testSlots.withKS(gains[3]);
        testSlots.withKV(gains[4]);
        testSlots.withKA(gains[5]);

        motorLeft.getConfigurator().apply(testSlots);
    }

    @Override
    public void stop() {
        runVolts(Volts.zero());
    }
    
}