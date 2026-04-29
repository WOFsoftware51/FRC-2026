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
import frc.robot.RobotState.Targets;
import frc.robot.util.LoggedTunableNumber;

public class ShooterSubsystem extends SubsystemBase {
    private ShooterIO io;
    ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

    private InterpolatingDoubleTreeMap treeMap = new InterpolatingDoubleTreeMap();
    private InterpolatingDoubleTreeMap treeMapFeed = new InterpolatingDoubleTreeMap();

    private double autonOffset = 0;
    public double teleopOffet = 0;

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

    double currentTarget = 0.0;

    public ShooterSubsystem(ShooterIO io) {
        this.io = io;

        treeMap.put(Inches.of(53.84).in(Meters), 2250.0);//
        treeMap.put(Inches.of(94.7).in(Meters), 2500.0);//
        treeMap.put(Inches.of(135.5).in(Meters), 2600.0);//
        treeMap.put(Inches.of(173.2).in(Meters), 2900.0);//
        treeMap.put(Inches.of(210.5).in(Meters), 3100.0);//
        treeMap.put(Inches.of(243.75).in(Meters), 3500.0);
        treeMap.put(Inches.of(280.0).in(Meters), 3900.0);


        treeMapFeed.put(Meters.of(4.03).in(Meters), 2400.0);
        treeMapFeed.put(Meters.of(5.70).in(Meters), 2900.0);
        treeMapFeed.put(Meters.of(7.03).in(Meters), 3250.0);
        treeMapFeed.put(Feet.of(27.0).in(Meters), 5000.0);


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
            autonOffset = 0; //50;
        }
        else {
            autonOffset = 0;
        }


        if(RobotState.getInstance().getCurrentTarget() == Targets.Hub) {
            currentTarget = treeMap.get(Double.valueOf(RobotState.getInstance().getFutureTurretToHub()));

        }
        else if(RobotState.getInstance().getCurrentTarget() == Targets.Feed) {
            currentTarget = treeMapFeed.get(Double.valueOf(RobotState.getInstance().getFutureTurretToHub()));
        }

        
        Logger.processInputs("Shooter", inputs);

        Logger.recordOutput("Shooter/atRPM", atRPM);
        Logger.recordOutput("Shooter/autonOffset", autonOffset);
        Logger.recordOutput("Shooter/teleopOffet", teleopOffet);

        Logger.recordOutput("Shooter/chassisShootingSpeed", chassisShootingSpeed);

        Logger.recordOutput("Shooter/TreeMap Angle", currentTarget);

    }

    public Command treeMapRPMCommand() {
        return run(() -> 
            {
                io.runVelocityRPM(
                    RPM.of(
                        currentTarget
                        +
                        autonOffset
                        +
                        teleopOffet
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
    public Command feeding() {
        return run(() -> 
            {
                io.runVelocityRPM(
                    RPM.of(
                        currentTarget
                    ) 
                );


                if (Math.abs(inputs.targetVelocity.in(RPM) - inputs.currentVelocity.in(RPM)) < 1000) {
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
