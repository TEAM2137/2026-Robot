package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.util.Alerts;
import frc.robot.util.Utils;

public class Climber extends SubsystemBase {
    public static class Constants {
        public static final double extendHeight = 10.0;
        public static final double retractHeight = 0.0;
    }

    private final ClimberIO io;
    private final ClimberIOInputsAutoLogged inputs;

    public Climber(ClimberIO io) {
        this.io = io;
        this.inputs = new ClimberIOInputsAutoLogged();
    }

    @Override
    public void periodic() {
        io.updateInputs(this.inputs);
        Logger.processInputs("Climber", this.inputs);
        Utils.logActiveCommand("Climber", this);

        Alerts.add("Climb motor disconnected", AlertType.kError, () -> !inputs.connected);
    }

    // TODO: this is VERY unrealistic
    public Command startClimbSequence(Trigger advance) {
        return new SequentialCommandGroup(
            this.extendClimb(),
            this.deployUpperHooks(),
            Commands.waitUntil(advance),
            this.retractClimb(),
            this.deployLowerHooks(),
            Commands.waitUntil(advance),
            this.retractUpperHooks(),
            this.extendClimb(),
            this.deployUpperHooks(),
            this.retractClimb(),
            this.deployLowerHooks(),
            Commands.waitUntil(advance),
            this.retractUpperHooks(),
            this.extendClimb(),
            this.deployUpperHooks(),
            this.retractClimb(),
            this.deployLowerHooks()
        )
        .withName("Climb Sequence");
    }

    public Command cancelClimbSequence() {
        return runOnce(() -> this.getCurrentCommand().cancel());
    }

    public Command extendClimb() {
        return runOnce(() -> io.setPosition(Constants.extendHeight));
    }

    public Command retractClimb() {
        return runOnce(() -> io.setPosition(Constants.retractHeight));
    }

    public Command deployUpperHooks() {
        return runOnce(() -> io.deployUpperHooks());
    }

    public Command deployLowerHooks() {
        return runOnce(() -> io.deployLowerHooks());
    }

    public Command retractUpperHooks() {
        return runOnce(() -> io.retractUpperHooks());
    }

    public Command retractLowerHooks() {
        return runOnce(() -> io.retractLowerHooks());
    }
}
