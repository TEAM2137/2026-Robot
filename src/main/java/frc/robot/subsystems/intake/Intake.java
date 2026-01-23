package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.intake.IntakeIO.IntakeIOInputs;

public class Intake extends SubsystemBase {
    private final IntakeIO io;
    private final IntakeIOInputs inputs;

    public Intake(IntakeIO io) {
        this.io = io;
        this.inputs = new IntakeIOInputs();
        Logger.processInputs("Intake", inputs);
    }

    public Command runRollers(double volts) {
        return runOnce(() -> io.runRollers(volts));
    }

    public Command deploy() {
        return runOnce(io::deploy);
    }

    public Command retract() {
        return runOnce(io::retract);
    }

    public Command startIntakeSequence() {
        return deploy().andThen(runRollers(12));
    }

    public Command stopIntakeSequence() {
        return retract().andThen(runRollers(0));
    }
}