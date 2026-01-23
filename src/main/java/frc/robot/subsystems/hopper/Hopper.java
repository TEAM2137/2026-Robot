package frc.robot.subsystems.hopper;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Hopper extends SubsystemBase {
    private final HopperIO io;

    public Hopper(HopperIO io) {
        this.io = io;
    }

    public Command runIndexer() {
        return runOnce(() -> io.runIndexer(12));
    }

    public Command stopIndexer() {
        return runOnce(() -> io.runIndexer(0));
    }
}
