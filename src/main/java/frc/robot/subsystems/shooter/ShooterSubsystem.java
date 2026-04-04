package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.*;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotState;
import frc.robot.util.LoggedTunableNumber;

public class ShooterSubsystem extends SubsystemBase {
    private ShooterIO io;
    ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

    private InterpolatingDoubleTreeMap treeMap = new InterpolatingDoubleTreeMap();

    private double autonOffset = 0;

    LoggedTunableNumber shooterRPM = new LoggedTunableNumber("Shooter/speed", 3000);
    
    LoggedTunableNumber kP = new LoggedTunableNumber("Shooter/kP", 0.0);
    LoggedTunableNumber kI = new LoggedTunableNumber("Shooter/kI", 0.0);
    LoggedTunableNumber kD = new LoggedTunableNumber("Shooter/kD", 0.0);
    LoggedTunableNumber kS = new LoggedTunableNumber("Shooter/kS", 0.0);
    LoggedTunableNumber kV = new LoggedTunableNumber("Shooter/kV", 0.0);
    LoggedTunableNumber kA = new LoggedTunableNumber("Shooter/kA", 0.0);

    public boolean gainsChanged = false;

    public boolean atRPM = false;
    public double chassisShootingSpeed = 1.0;

    public ShooterSubsystem(ShooterIO io) {
        this.io = io;

        treeMap.put(Inches.of(68.2).in(Meters), 2175.0);
        treeMap.put(Inches.of(87.4).in(Meters), 2350.0);
        treeMap.put(Inches.of(107.0).in(Meters), 2600.0);
        treeMap.put(Inches.of(127.0).in(Meters), 2850.0);
        treeMap.put(Inches.of(147.6).in(Meters), 3000.0);
        treeMap.put(Inches.of(166.6).in(Meters), 2900.0);
        treeMap.put(Inches.of(184.0).in(Meters), 2900.0);
        treeMap.put(Inches.of(208.0).in(Meters), 2925.0);
    }

    public Supplier<Double> getChassisShootingSpeed() {
        return () -> chassisShootingSpeed;
    }


    @Override
    public void periodic() {
        io.updateInputs(inputs);


        if(
            kP.hasChanged(kP.hashCode()) ||
            kI.hasChanged(kI.hashCode()) ||
            kD.hasChanged(kD.hashCode()) ||
            kV.hasChanged(kV.hashCode()) ||
            kA.hasChanged(kA.hashCode()) ||
            kS.hasChanged(kS.hashCode())
        ) {
            gainsChanged = true;
        }
        else {
            gainsChanged = false;
        }



        if(inputs.currentVelocity.in(RPM)>100) {
            chassisShootingSpeed = 0.5;
        }
        else {
            chassisShootingSpeed = 1.0;
        }

        if(DriverStation.isAutonomous()) {
            autonOffset = 50;
        }
        else{
            autonOffset = 0;
        }

        
        Logger.processInputs("Shooter", inputs);

        Logger.recordOutput("Shooter/atRPM", atRPM);
        Logger.recordOutput("Shooter/autonOffset", autonOffset);

        Logger.recordOutput("Shooter/chassisShootingSpeed", chassisShootingSpeed);

        Logger.recordOutput("Shooter/TreeMap Angle", treeMap.get(Double.valueOf(RobotState.getInstance().getTurretToHub())));

    }

    public Command treeMapRPMCommand() {
        return run(() -> 
            {
                io.runVelocityRPM(
                    RPM.of(
                        treeMap.get(
                            Double.valueOf(
                                RobotState.getInstance().getTurretToHub()
                            )
                            +
                            autonOffset
                        )
                    ) 
                );


                if (Math.abs(inputs.targetVelocity.in(RPM) - inputs.currentVelocity.in(RPM)) < 50) {
                    atRPM = true;
                }

            }
        )
        .finallyDo(() ->
            {
                io.stop();
                atRPM = false;
            }
        );
        

    }

    public Command runRPMCommand() {
        return run(() ->
            {io.runVelocityRPM(RPM.of(shooterRPM.get()));

            if (Math.abs(inputs.targetVelocity.in(RPM) - inputs.currentVelocity.in(RPM)) < 50) {
                    atRPM = true;
            }}

        )
        .finallyDo(() ->
            {io.stop();
            atRPM = false;}

        );
    }

    public Command runRPMCommand(double rpm) {
        return run(() ->
            {io.runVelocityRPM(RPM.of(rpm));
                
            if (Math.abs(inputs.targetVelocity.in(RPM) - inputs.currentVelocity.in(RPM)) < 50) {
                atRPM = true;
            }}

        )
        .finallyDo(() ->
            {io.stop();
            atRPM = false;}
        );
    }

    public Command updateGainsCommand() {
        return runOnce(() ->
            {
                io.updateGains(
                    kP.get(),
                    kI.get(), 
                    kD.get(),  
                    kS.get(), 
                    kV.get(), 
                    kA.get()
                );
                System.out.println("Shooter Gains Updated!");
            }
        );
    }


}
