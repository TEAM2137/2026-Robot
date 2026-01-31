package frc.robot.subsystems.launcher.flywheel;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.RobotContainer;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.FieldConstants;

public class Flywheel {
    private final FlywheelIO io;
    private final FlywheelIOInputsAutoLogged inputs;

    public Flywheel(FlywheelIO io) {
        this.io = io;
        this.inputs = new FlywheelIOInputsAutoLogged();
    }

    public void setRPM(double rpm) {
        io.setRPM(rpm);
    }

    public void setVoltage(double volts) {
        io.setVoltage(volts);
    }
    
    public boolean canFire() {
        Translation2d robotPos = RobotContainer.getInstance().drive.getPose().getTranslation();
        if (AllianceFlipUtil.shouldFlip()) robotPos = AllianceFlipUtil.flip(robotPos);
        return !FieldConstants.noFireZone1.contains(robotPos) && !FieldConstants.noFireZone2.contains(robotPos);
    }

    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Launcher/Flywheel", inputs);
    }
}
