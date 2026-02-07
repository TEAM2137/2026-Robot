package frc.robot.subsystems.indexer;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.Utils;

public class Indexer extends SubsystemBase {
    private final IndexerIO io;
    private final IndexerIOInputsAutoLogged inputs;

    public Indexer(IndexerIO io) {
        this.io = io;
        this.inputs = new IndexerIOInputsAutoLogged();
    }

    @Override
    public void periodic() {
        io.updateInputs(this.inputs);
        Logger.processInputs("Indexer", this.inputs);
        Utils.logActiveCommand("Indexer", this);
    }

    public Command run() {
        return runOnce(() -> {
            io.runIndexer(12);
            io.runFeeder(3);
        });
    }

    public Command stop() {
        return runOnce(() -> {
            io.runIndexer(0);
            io.runFeeder(0);
        });
    }
}
