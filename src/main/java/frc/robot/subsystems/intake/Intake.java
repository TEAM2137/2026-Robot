package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.Alerts;
import frc.robot.util.Utils;

public class Intake extends SubsystemBase {
    public static class Constants {
        public static final double homePosition = 0.0;
        public static final double deployPosition = 10.0;
        public static final double intakeVoltage = 6.0;
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

    public Command runRollers(double volts) {
        return runOnce(() -> io.runRollers(volts));
    }

    public Command deploy() {
        return runOnce(() -> io.setPosition(Constants.deployPosition));
    }

    public Command retract() {
        return runOnce(() -> io.setPosition(Constants.homePosition));
    }

    public Command startIntakeSequence() {
        return deploy().andThen(runRollers(Constants.intakeVoltage));
    }

    public Command stopIntakeSequence() {
        return retract().andThen(runRollers(0));
    }
}