package frc.robot.subsystems.launcher;

import java.util.Optional;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

    private AllianceStationID lastAllianceStation = AllianceStationID.Unknown;

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

        double passingFlipY = this.getPassingFlipY();
        if (robotPos.getX() < FieldConstants.allianceZoneX) this.shotCalculator = ShotCalculator.SOTF_HUB;
        else if (robotPos.getY() < passingFlipY) this.shotCalculator = ShotCalculator.PASS_RIGHT;
        else this.shotCalculator = ShotCalculator.PASS_LEFT;

        ShotParameters params = this.shotCalculator.calculate(robot);
        boolean isTest = DriverStation.isTest();
        TestMode testMode = RobotContainer.getInstance().getTestMode();

        if (isTest && testMode == TestMode.TURRET) {
            this.turret.setAngleFieldRelative(Rotation2d.fromRadians(Math.atan2(
                robot.operatorController.getRightY(),
                -robot.operatorController.getRightX()
            )));
            this.hood.setAngle(0);
            this.flywheel.setRPM(0);
        }
        else if (isTest && testMode == TestMode.LOOKUP_TABLES) {
            this.turret.setAngleFieldRelative(params.turretAngle());
            this.hood.setAngle(this.manualHoodAngle);
            this.flywheel.setRPM(this.manualFlywheelRPM);
            Logger.recordOutput("LookupTables/HoodAngle", this.manualHoodAngle);
            Logger.recordOutput("LookupTables/FlywheelRPM", this.manualFlywheelRPM);
        }
        else if (!isTest) {
            this.turret.setAngleFieldRelative(params.turretAngle());
            this.hood.setAngle(params.hoodAngle());
            if (this.isLaunching) this.flywheel.setRPM(params.flywheelRpm());
        }

        turret.periodic();
        hood.periodic();
        flywheel.periodic();

        Logger.recordOutput("Launcher/IsLaunching", this.isLaunching);
        Utils.logActiveCommand("Launcher", this);
    }

    public double getPassingFlipY() {
        AllianceStationID currentStation = DriverStation.getRawAllianceStation();
        if (!SmartDashboard.containsKey("PassingPriority")) {
            this.lastAllianceStation = currentStation;
            SmartDashboard.putBoolean("PassingPriority", false);
        }
        if (this.lastAllianceStation != currentStation) {
            this.lastAllianceStation = currentStation;
            this.getDefaultPassingPriority().ifPresent(priority ->
                SmartDashboard.putBoolean("PassingPriority", priority));
        }
        boolean priority = SmartDashboard.getBoolean("PassingPriority", false);
        if (AllianceFlipUtil.shouldFlip()) return priority ? FieldConstants.midPassY2 : FieldConstants.midPassY1;
        return priority ? FieldConstants.midPassY1 : FieldConstants.midPassY2;
    }

    public Optional<Boolean> getDefaultPassingPriority() {
        if (DriverStation.getRawAllianceStation().toString().contains("1")) return Optional.of(false);
        if (DriverStation.getRawAllianceStation().toString().contains("3")) return Optional.of(true);
        return Optional.empty();
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

    public boolean shouldLimitDrive() {
        return this.isLaunching && this.shotCalculator == ShotCalculator.SOTF_HUB;
    }
}
