package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.*;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotState;
import frc.robot.util.LoggedTunableNumber;

public class ShooterSubsystem extends SubsystemBase {
    private ShooterIO io;
    ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

    private InterpolatingDoubleTreeMap treeMap = new InterpolatingDoubleTreeMap();

    LoggedTunableNumber shooterRPM = new LoggedTunableNumber("Shooter/speed", 3000);
    
    LoggedTunableNumber kP = new LoggedTunableNumber("Shooter/kP", 0.0);
    LoggedTunableNumber kI = new LoggedTunableNumber("Shooter/kI", 0.0);
    LoggedTunableNumber kD = new LoggedTunableNumber("Shooter/kD", 0.0);
    LoggedTunableNumber kS = new LoggedTunableNumber("Shooter/kS", 0.0);
    LoggedTunableNumber kV = new LoggedTunableNumber("Shooter/kV", 0.0);
    LoggedTunableNumber kA = new LoggedTunableNumber("Shooter/kA", 0.0);

    public boolean gainsChanged = false;

    public boolean atRPM = false;

    public ShooterSubsystem(ShooterIO io) {
        this.io = io;

        treeMap.put(Inches.of(101.90).in(Meters), 2600.0);
        treeMap.put(Inches.of(144.50).in(Meters), 2750.0);
        treeMap.put(Inches.of(182.38).in(Meters), 3120.0);
        treeMap.put(Inches.of(207.28).in(Meters), 3200.0);
        treeMap.put(Inches.of(66.000).in(Meters), 2300.0);
        treeMap.put(Inches.of(77.8).in(Meters), 2380.0);
        treeMap.put(Inches.of(91.2).in(Meters), 2450.0);
        treeMap.put(Inches.of(116.0).in(Meters), 2500.0);
        treeMap.put(Inches.of(134.7).in(Meters), 2600.0);
        treeMap.put(Inches.of(160.3).in(Meters), 2800.0);
        treeMap.put(Inches.of(172.3).in(Meters), 3000.0);
        treeMap.put(Inches.of(213).in(Meters), 3350.0);

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


        
        Logger.processInputs("Shooter", inputs);

        Logger.recordOutput("Shooter/TreeMap Angle", treeMap.get(Double.valueOf(RobotState.getInstance().getDistanceFromHubMeters())));

    }

    public Command treeMapRPMCommand() {
        return run(() -> 
            {
                io.runVelocityRPM(
                    RPM.of(
                        treeMap.get(
                            Double.valueOf(
                                RobotState.getInstance().getDistanceFromHubMeters()
                            )
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
        return Commands.run(() ->
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
