package frc.robot.subsystems.launcher;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Launcher extends SubsystemBase {
    private final LauncherIO io;

    public Launcher(LauncherIO io) {
        this.io = io;
    }

    public Command setFlywheelSpeed(double rpm) {
        return runOnce(() -> io.setFlywheelSpeed(rpm));
    }

    public Command setHoodAngle(double degrees) {
        return runOnce(() -> io.setHoodAngle(degrees));
    }

    public Command setTurretAngle(double degrees) {
        return runOnce(() -> io.setTurretAngle(degrees));
    }
}
