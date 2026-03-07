package frc.robot.subsystems.launcher.flywheel;

import java.util.function.BooleanSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Alert.AlertType;
import frc.robot.RobotContainer;
import frc.robot.util.Alerts;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.FieldConstants;

public class Flywheel {
    private final FlywheelIO io;
    private final FlywheelIOInputsAutoLogged inputs;

    public Flywheel(FlywheelIO io) {
        this.io = io;
        this.inputs = new FlywheelIOInputsAutoLogged();

        Alerts.add("Flywheel motors disconnected", AlertType.kError, () -> !inputs.flywheelConnected);
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
        return !FieldConstants.noFireZone.contains(robotPos);
    }

    public BooleanSupplier isWithinTarget(double range) {
        return () -> this.io.isWithinTarget(range);
    }

    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Launcher/Flywheel", inputs);
    }
}
