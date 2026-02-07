package frc.robot.subsystems.launcher.hood;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.Alert.AlertType;
import frc.robot.util.Alerts;

public class Hood {
    private final HoodIO io;
    private final HoodIOInputsAutoLogged inputs;

    public Hood(HoodIO io) {
        this.io = io;
        this.inputs = new HoodIOInputsAutoLogged();

        Alerts.add("Hood motor disconnected", AlertType.kError, () -> !inputs.connected);
    }

    public void setAngle(double degrees) {
        io.setAngle(degrees);
    }

    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Launcher/Hood", inputs);
    }
}
