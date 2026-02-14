package frc.robot.subsystems.launcher;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.RobotContainer;
import frc.robot.subsystems.launcher.ShotCalculator.ShotParameters;
import frc.robot.subsystems.launcher.flywheel.Flywheel;
import frc.robot.subsystems.launcher.flywheel.FlywheelIO;
import frc.robot.subsystems.launcher.hood.Hood;
import frc.robot.subsystems.launcher.hood.HoodIO;
import frc.robot.subsystems.launcher.turret.Turret;
import frc.robot.subsystems.launcher.turret.TurretIO;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.FieldConstants;
import frc.robot.util.TestMode;
import frc.robot.util.Utils;

public class Launcher extends SubsystemBase {
    private final Turret turret;
    private final Hood hood;
    private final Flywheel flywheel;

    private final Trigger isLaunchingTrigger;
    private boolean isLaunching = false;

    private ShotCalculator shotCalculator;

    // lookup table tuning values
    private double manualHoodAngle;
    private double manualFlywheelRPM;

    public Launcher(TurretIO turretIO, HoodIO hoodIO, FlywheelIO flywheelIO) {
        this.turret = new Turret(turretIO);
        this.hood = new Hood(hoodIO);
        this.flywheel = new Flywheel(flywheelIO);

        this.shotCalculator = ShotCalculator.HUB;

        this.isLaunchingTrigger = new Trigger(() -> this.isLaunching);
        this.isLaunchingTrigger.onFalse(this.setFlywheelSpeed(0));

        RobotModeTriggers.teleop().onFalse(this.runOnce(() -> this.isLaunching = false));
    }

    public Trigger isLaunching() {
        return this.isLaunchingTrigger;
    }

    public Command startLaunching() {
        return runOnce(() -> this.setIsLaunching(true));
    }

    public Command stopLaunching() {
        return runOnce(() -> this.setIsLaunching(false));
    }

    public void setIsLaunching(boolean isLaunching) {
        this.isLaunching = isLaunching;
    }

    @Override
    public void periodic() {
        RobotContainer robot = RobotContainer.getInstance();

        Translation2d robotPos = robot.drive.getPose().getTranslation();
        if (AllianceFlipUtil.shouldFlip()) robotPos = new Translation2d(AllianceFlipUtil.flipX(robotPos.getX()), robotPos.getY());
        if (robotPos.getX() < FieldConstants.allianceZoneX) this.shotCalculator = ShotCalculator.HUB;
        else if (robotPos.getY() < FieldConstants.passingFlipY) this.shotCalculator = ShotCalculator.PASS_RIGHT;
        else this.shotCalculator = ShotCalculator.PASS_LEFT;

        ShotParameters params = this.shotCalculator.calculate(robot);
        TestMode testMode = RobotContainer.getInstance().getTestMode();

        if (testMode == TestMode.TURRET) {
            this.turret.setAngleFieldRelative(Rotation2d.fromRadians(Math.atan2(
                robot.operatorController.getRightY(),
                robot.operatorController.getRightX()
            )));
            this.hood.setAngle(0);
            this.flywheel.setRPM(0);
        }
        else if (testMode == TestMode.LOOKUP_TABLES) {
            this.turret.setAngleFieldRelative(params.turretAngle());
            this.hood.setAngle(this.manualHoodAngle);
            this.flywheel.setRPM(this.manualFlywheelRPM);
        }
        else {
            this.turret.setAngleFieldRelative(params.turretAngle());
            if (testMode != TestMode.HOOD) this.hood.setAngle(params.hoodAngle());
            if (this.isLaunching) this.flywheel.setRPM(params.flywheelRpm());
        }

        turret.periodic();
        hood.periodic();
        flywheel.periodic();

        Logger.recordOutput("Launcher/IsLaunching", this.isLaunching);
        Utils.logActiveCommand("Launcher", this);
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

    public Command modifyManualHoodAngle(int degreesPerSec) {
        return run(() -> this.manualHoodAngle += degreesPerSec / 50.0);
    }

    public Command modifyManualFlywheelRPM(int rpmPerSec) {
        return run(() -> this.manualFlywheelRPM += rpmPerSec / 50.0);
    }
}
