package frc.robot.subsystems.hopper;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Hopper extends SubsystemBase {
    private final HopperIO io;
    private final HopperIOInputsAutoLogged inputs;

    public Hopper(HopperIO io) {
        this.io = io;
        this.inputs = new HopperIOInputsAutoLogged();
    }

    @Override
    public void periodic() {
        io.updateInputs(this.inputs);
        Logger.processInputs("Hopper", this.inputs);
    }

    public Command run() {
        return runOnce(() -> {
            io.runIndexer(12);
            io.runFeeder(12);
        });
    }

    public Command stop() {
        return runOnce(() -> {
            io.runIndexer(0);
            io.runFeeder(0);
        });
    }
}
