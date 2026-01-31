package frc.robot.subsystems.launcher;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.launcher.ShotCalculator.ShotParameters;
import frc.robot.subsystems.launcher.flywheel.Flywheel;
import frc.robot.subsystems.launcher.flywheel.FlywheelIO;
import frc.robot.subsystems.launcher.hood.Hood;
import frc.robot.subsystems.launcher.hood.HoodIO;
import frc.robot.subsystems.launcher.turret.Turret;
import frc.robot.subsystems.launcher.turret.TurretIO;
import frc.robot.util.FieldConstants;

public class Launcher extends SubsystemBase {
    private final Turret turret;
    private final Hood hood;
    private final Flywheel flywheel;

    private ShotCalculator shotCalculator;

    public Launcher(TurretIO turretIO, HoodIO hoodIO, FlywheelIO flywheelIO) {
        this.turret = new Turret(turretIO);
        this.hood = new Hood(hoodIO);
        this.flywheel = new Flywheel(flywheelIO);

        this.shotCalculator = ShotCalculator.HUB;
    }

    @Override
    public void periodic() {
        RobotContainer robot = RobotContainer.getInstance();

        Translation2d robotPos = robot.drive.getPose().getTranslation();
        if (robotPos.getX() < FieldConstants.allianceZoneX) this.shotCalculator = ShotCalculator.HUB;
        else if (robotPos.getY() < FieldConstants.passingFlipY) this.shotCalculator = ShotCalculator.HUB;
        else this.shotCalculator = ShotCalculator.HUB;

        ShotParameters params = this.shotCalculator.calculate(robot);
        this.turret.setAngleFieldRelative(params.turretAngle());
        this.flywheel.setRPM(params.flywheelRpm());
        this.hood.setAngle(params.hoodAngle());

        turret.periodic();
        hood.periodic();
        flywheel.periodic();
    }

    public Turret getTurret() {
        return this.turret;
    }

    public Hood getHood() {
        return this.hood;
    }

    public Flywheel getFlywheel() {
        return this.flywheel;
    }

    public Command setFlywheelSpeed(DoubleSupplier rpm) {
        return runOnce(() -> flywheel.setRPM(rpm.getAsDouble()));
    }

    public Command setFlywheelSpeed(double rpm) {
        return runOnce(() -> flywheel.setRPM(rpm));
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
}
