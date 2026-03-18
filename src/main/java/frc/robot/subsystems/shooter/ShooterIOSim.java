package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Volts;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.AbstractDriveTrainSimulation;
import org.ironmaple.simulation.seasonspecific.rebuilt2026.RebuiltFuelOnFly;
import org.ironmaple.utils.FieldMirroringUtils;
import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants;
import frc.robot.RobotState;

public class ShooterIOSim implements ShooterIO{
    private TalonFX motorLeft = new TalonFX(Constants.ShooterConstants.kMotorLeftID, Constants.kCANIvoreName);
    private TalonFX motorRight = new TalonFX(Constants.ShooterConstants.kMotorRightID, Constants.kCANIvoreName);

    private TalonFXSimState motorLeftSim = motorLeft.getSimState();
    private TalonFXSimState motorRightSim = motorRight.getSimState();

    private AbstractDriveTrainSimulation mapleSim;

    private DCMotorSim sim = new DCMotorSim(
        LinearSystemId.createDCMotorSystem(
            DCMotor.getKrakenX60(2),
            0.01, 
            Constants.ShooterConstants.kGearRatio
        ), 
        DCMotor.getKrakenX60(2)
    );
    
    private VelocityVoltage speed = new VelocityVoltage(0);
    private double speeds;

    private TalonFXConfiguration configs = new TalonFXConfiguration();

    private double targetVelocity = 0;

    private double kP = 0.8;
    private double kI = 0;
    private double kD = 0;
    private double kV = 0.12;
    private double kA = 0.2;
    private double kS = 0.0;


    public ShooterIOSim(AbstractDriveTrainSimulation mapleSim) {    
        this.mapleSim = mapleSim;

        configs.MotorOutput.withInverted(InvertedValue.CounterClockwise_Positive);

        configs.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        configs.Slot0.kP = kP;
        configs.Slot0.kI = kI;
        configs.Slot0.kD = kD;
        configs.Slot0.kV = kV;
        configs.Slot0.kA = kA;
        configs.Slot0.kS = kS;


        motorLeft.getConfigurator().apply(configs);

        motorRight.setControl(new Follower(Constants.ShooterConstants.kMotorLeftID, MotorAlignmentValue.Opposed));
    }

    RebuiltFuelOnFly fuelOnFly;


    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        motorLeftSim.setSupplyVoltage(12.0);
        double volts = motorLeftSim.getMotorVoltage();
        sim.setInputVoltage(volts);

        sim.update(0.02);
        double shooterRotations = sim.getAngularPositionRotations();
        double shooterRPM = sim.getAngularVelocityRPM();
        double shooterRPMPerMinute = sim.getAngularAcceleration().in(RotationsPerSecondPerSecond)*60;
        
        double rotorRotations = shooterRotations * Constants.ShooterConstants.kGearRatio;
        double rotorRPS = (RPM.of(shooterRPM).in(RotationsPerSecond)) * Constants.ShooterConstants.kGearRatio;
        double rotorRPSPerSecond = ((shooterRPMPerMinute)/60) * Constants.ShooterConstants.kGearRatio;
        
        motorLeftSim.setRawRotorPosition(rotorRotations);
        motorLeftSim.setRotorVelocity(rotorRPS);
        motorLeftSim.setRotorAcceleration(rotorRPSPerSecond);

        motorRightSim.setRawRotorPosition(-rotorRotations);
        motorRightSim.setRotorVelocity(-rotorRPS);
        motorRightSim.setRotorAcceleration(-rotorRPSPerSecond);


        inputs.currentVelocity.mut_replace(shooterRPM, RPM);
        speeds = shooterRPM;
        inputs.targetVelocity.mut_replace(RotationsPerSecond.of(targetVelocity).in(RPM), RPM);

        // inputs.currentAcceleration.mut_replace(shooterRPMPerMinute, RPM);

        inputs.appliedVoltage.mut_replace(volts, Volts);

        inputs.supplyCurrent.mut_replace(sim.getCurrentDrawAmps(), Amps);
        inputs.torqueCurrent.mut_replace(sim.getCurrentDrawAmps(), Amps);

        updateBall();
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

        shootBall();
    }

    @Override
    public void updateGains(double... gains) {
        // kP = gains[0];
        // kI = gains[1];
        // kD = gains[2];
        // kS = gains[3];
        // kV = gains[4];
        // kA = gains[5];

        // this.pidController.setPID(kP, kI, kD);
        // this.ffController.setKs(kS);
        // this.ffController.setKv(kV);
        // this.ffController.setKa(kA);
    }

    @Override
    public void stop() {
        runVolts(Volts.zero());
    }

    boolean ballLaunching = false;
    Translation3d ballTargetPosition = new Translation3d(0.25, 5.56, 2.3);
    private void shootBall() {
            fuelOnFly = new RebuiltFuelOnFly(
            // Specify the position of the chassis when the note is launched
            mapleSim.getSimulatedDriveTrainPose().getTranslation(),
            // Specify the translation of the shooter from the robot center (in the shooter’s reference frame)
            RobotState.getInstance().getRobotToTurret().getTranslation().toTranslation2d(),
            // Specify the field-relative speed of the chassis, adding it to the initial velocity of the projectile
            RobotState.getInstance().getChassisSpeeds(),
            // The shooter facing direction is the same as the robot’s facing direction
            mapleSim.getSimulatedDriveTrainPose().getRotation().plus(
                // Add the shooter’s rotation
                // RobotState.getInstance().getRobotToTurret().getRotation().toRotation2d()
                new Rotation2d(Degrees.of(90))
                ),
            // Initial height of the flying note
            RobotState.getInstance().getRobotToTurret().getTranslation().getMeasureZ(), 
            // The launch speed is proportional to the RPM; assumed to be 16 meters/second at 6000 RPM
            // LinearVelocity.ofBaseUnits(speed.getVelocityMeasure().times(20/6000).in(RPM), MetersPerSecond),
            LinearVelocity.ofBaseUnits(speeds*17/6000, MetersPerSecond),
            // The angle at which the note is launched
            Degrees.of(60)
        );

        fuelOnFly
        // Set the target center to the Rebbuilt Hub of the current alliance
        .withTargetPosition(() -> FieldMirroringUtils.toCurrentAllianceTranslation(ballTargetPosition))
        // Set the tolerance: x: ±0.5m, y: ±1.2m, z: ±0.3m (this is the size of the speaker's "mouth")
        .withTargetTolerance(new Translation3d(0.5, 1.2, 0.3))
        // Set a callback to run when the fuel hits the target
        .withHitTargetCallBack(() -> System.out.println("Hit hub, +1 point!"));

        fuelOnFly.launch();

        SimulatedArena.getInstance().addGamePieceProjectile(fuelOnFly); 
    }


    private double time;
    private void updateBall() {
        // Add the projectile to the simulated arena

        // if(Timer.getFPGATimestamp()-time>3) {
        //     SimulatedArena.getInstance().removeProjectile(fuelOnFly); 
        //     time = Timer.getFPGATimestamp();
        //     System.out.print("RESET BALLSSSS");
        // }

        if(Timer.getFPGATimestamp()-time>5) {
            SimulatedArena.getInstance().clearGamePieces(); 
            time = Timer.getFPGATimestamp();
            System.out.print("RESET BALLSSSS");
        }

        Logger.recordOutput("Shooter/Speeds?", speeds, MetersPerSecond);
    }

}
