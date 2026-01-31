package frc.robot.subsystems.launcher.hood;

import org.littletonrobotics.junction.Logger;

public class Hood {
    private final HoodIO io;
    private final HoodIOInputsAutoLogged inputs;

    public Hood(HoodIO io) {
        this.io = io;
        this.inputs = new HoodIOInputsAutoLogged();
    }

    public void setAngle(double degrees) {
        io.setAngle(degrees);
    }

    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Launcher/Hood", inputs);
    }
}
