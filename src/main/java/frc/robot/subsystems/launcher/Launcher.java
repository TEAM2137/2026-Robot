package frc.robot.subsystems.launcher;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Launcher extends SubsystemBase {
    private final TurretIO turret;
    private final HoodIO hood;
    private final FlywheelIO flywheel;

    private final TurretIOInputsAutoLogged turretInputs;
    private final HoodIOInputsAutoLogged hoodInputs;
    private final FlywheelIOInputsAutoLogged flywheelInputs;

    public Launcher(TurretIO turret, HoodIO hood, FlywheelIO flywheel) {
        this.turret = turret;
        this.hood = hood;
        this.flywheel = flywheel;

        this.turretInputs = new TurretIOInputsAutoLogged();
        this.hoodInputs = new HoodIOInputsAutoLogged();
        this.flywheelInputs = new FlywheelIOInputsAutoLogged();
    }

    @Override
    public void periodic() {
        turret.updateInputs(this.turretInputs);
        hood.updateInputs(this.hoodInputs);
        flywheel.updateInputs(this.flywheelInputs);

        Logger.processInputs("Launcher/Turret", this.turretInputs);
        Logger.processInputs("Launcher/Hood", this.hoodInputs);
        Logger.processInputs("Launcher/Flywheel", this.flywheelInputs);
    }

    public Command setFlywheelSpeed(double rpm) {
        return runOnce(() -> flywheel.setSpeed(rpm));
    }

    public Command setHoodAngle(double degrees) {
        return runOnce(() -> hood.setAngle(degrees));
    }

    public Command setTurretAngle(Rotation2d angle) {
        return runOnce(() -> turret.setAngleFieldRelative(angle));
    }

    public Command setShotCalculator(ShotCalculator calculator) {
        return runOnce(() -> turret.setShotCalculator(calculator));
    }
}
