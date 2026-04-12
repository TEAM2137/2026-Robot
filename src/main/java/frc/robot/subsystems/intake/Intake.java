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
        public static final double deployPosition = 2.329;//1.995;
        public static final double rollerRPM = 4800;
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

    public Command setRollerVoltage(DoubleSupplier volts) {
        return runOnce(() -> io.setRollerVoltage(volts.getAsDouble()));
    }

    public Command setRollerVoltage(double volts) {
        return runOnce(() -> io.setRollerVoltage(volts));
    }

    public Command setPivotVoltage(double volts) {
        return runOnce(() -> io.setPivotVoltage(volts));
    }

    public Command runRollers() {
        // return runOnce(() -> io.setRollerRPM(Constants.rollerRPM));
        return runOnce(() -> io.setRollerVoltage(12));
    }

    public Command stopRollers() {
        return this.setRollerVoltage(0);
    }

    public Command agitate() {
        return this.agitate(0.7);
    }

    public Command agitate(double deployPercent) {
        return this.runRollers()
            .andThen(new SequentialCommandGroup(
                runOnce(() -> io.setPosition(Constants.homePosition)),
                Commands.waitSeconds(0.6),
                runOnce(() -> io.setPosition(Constants.deployPosition * deployPercent)),
                Commands.waitSeconds(0.6)
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
        return deploy().andThen(runRollers())
            .withName("Intake Deploy Sequence");
    }

    public Command stopIntakeSequence() {
        return Commands.waitSeconds(0.5).andThen(stopRollers())
            .withName("Intake Retract Sequence");
    }

    public Command resetPosition() {
        return runOnce(() -> this.io.resetPosition());
    }
}