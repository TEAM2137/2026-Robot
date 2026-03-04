package frc.robot.subsystems.intake;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.Alerts;
import frc.robot.util.Utils;

public class Intake extends SubsystemBase {
    public static class Constants {
        public static final double homePosition = 0.0;
        public static final double halfwayPosition = 1.0;
        public static final double deployPosition = 2.2;//1.995;
        public static final double rollerVoltage = 8.0;
    }

    private final IntakeIO io;
    private final IntakeIOInputsAutoLogged inputs;

    public Intake(IntakeIO io) {
        this.io = io;
        this.inputs = new IntakeIOInputsAutoLogged();

        Alerts.add("Intake pivot disconnected", AlertType.kError, () -> !inputs.pivotConnected);
        Alerts.add("Intake rollers disconnected", AlertType.kError, () -> !inputs.rollersConnected);
    }

    @Override
    public void periodic() {
        io.updateInputs(this.inputs);
        Logger.processInputs("Intake", this.inputs);
        Utils.logActiveCommand("Intake", this);
    }

    public Command runRollers(DoubleSupplier volts) {
        return runOnce(() -> io.runRollers(volts.getAsDouble()));
    }

    public Command runRollers(double volts) {
        return runOnce(() -> io.runRollers(volts));
    }

    public Command agitate() {
        return runRollers(Constants.rollerVoltage)
            .andThen(new SequentialCommandGroup(
                runOnce(() -> io.setPosition(Constants.halfwayPosition)),
                Commands.waitSeconds(0.4),
                runOnce(() -> io.setPosition(Constants.homePosition)),
                Commands.waitSeconds(0.4)
            ).repeatedly());
    }

    public Command deploy() {
        return runOnce(() -> io.setPosition(Constants.deployPosition));
    }

    public Command retract() {
        return runOnce(() -> io.setPosition(Constants.homePosition));
            // .andThen(Commands.waitSeconds(1))
            // .andThen(runOnce(() -> io.holdCurrentPosition()));
    }

    public Command startIntakeSequence() {
        return deploy().andThen(runRollers(Constants.rollerVoltage))
            .withName("Intake Deploy Sequence");
    }

    public Command stopIntakeSequence() {
        return Commands.waitSeconds(0.5).andThen(runRollers(0))
            .withName("Intake Retract Sequence");
    }
}