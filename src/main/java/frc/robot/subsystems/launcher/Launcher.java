package frc.robot.subsystems.launcher;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Launcher extends SubsystemBase {
    private final Turret turret;
    private final HoodIO hood;
    private final FlywheelIO flywheel;

    private final HoodIOInputsAutoLogged hoodInputs;
    private final FlywheelIOInputsAutoLogged flywheelInputs;

    public Launcher(TurretIO turretIO, HoodIO hood, FlywheelIO flywheel) {
        this.turret = new Turret(turretIO);
        this.hood = hood;
        this.flywheel = flywheel;

        this.hoodInputs = new HoodIOInputsAutoLogged();
        this.flywheelInputs = new FlywheelIOInputsAutoLogged();
    }

    @Override
    public void periodic() {
        turret.periodic();
        hood.updateInputs(this.hoodInputs);
        flywheel.updateInputs(this.flywheelInputs);

        Logger.processInputs("Launcher/Hood", this.hoodInputs);
        Logger.processInputs("Launcher/Flywheel", this.flywheelInputs);
    }

    public Command setFlywheelSpeed(DoubleSupplier rpm) {
        return runOnce(() -> flywheel.setSpeed(rpm.getAsDouble()));
    }

    public Command setFlywheelSpeed(double rpm) {
        return runOnce(() -> flywheel.setSpeed(rpm));
    }

    public Command setFlywheelVoltage(DoubleSupplier volts) {
        return runOnce(() -> flywheel.setVoltage(volts.getAsDouble()));
    }

    public Command setFlywheelVoltage(double volts) {
        return runOnce(() -> flywheel.setVoltage(volts));
    }

    public Command setHoodAngle(DoubleSupplier degrees) {
        return runOnce(() -> hood.setAngle(degrees.getAsDouble()));
    }

    public Command setHoodAngle(double degrees) {
        return runOnce(() -> hood.setAngle(degrees));
    }

    public Command setTurretAngle(Supplier<Rotation2d> angle) {
        return runOnce(() -> turret.setAngleFieldRelative(angle.get()));
    }

    public Command setTurretAngle(Rotation2d angle) {
        return runOnce(() -> turret.setAngleFieldRelative(angle));
    }

    public Command setShotCalculator(ShotCalculator calculator) {
        // TODO move to dedicated turret class
        return Commands.none();
    }
}
