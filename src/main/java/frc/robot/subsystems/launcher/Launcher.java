package frc.robot.subsystems.launcher;

import java.util.Optional;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.math.geometry.Pose2d;
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
import frc.robot.util.ShiftInfo;
import frc.robot.util.TestMode;
import frc.robot.util.Utils;

public class Launcher extends SubsystemBase {
    private final Turret turret;
    private final Hood hood;
    private final Flywheel flywheel;

    private final Trigger isLaunching;
    private final Trigger inAllianceZone;
    private final Trigger inAllianceZoneDebounced;
    private final Trigger inNeutralZone;
    private final Trigger inNeutralZoneDebounced;

    private LaunchState state = LaunchState.AUTOMATIC;
    private boolean autofire = false;

    private double manualHoodAngle;
    private double manualFlywheelRPM;

    private ShotCalculator shotCalculator;

    private AllianceStationID lastAllianceStation = AllianceStationID.Unknown;

    public Launcher(TurretIO turretIO, HoodIO hoodIO, FlywheelIO flywheelIO) {
        this.turret = new Turret(turretIO);
        this.hood = new Hood(hoodIO);
        this.flywheel = new Flywheel(flywheelIO);

        this.shotCalculator = ShotCalculator.HUB;

        this.isLaunching = new Trigger(() -> this.state == LaunchState.LAUNCH || (this.autofire && state != LaunchState.DONT_LAUNCH));

        this.inAllianceZone = new Trigger(() -> this.shotCalculator == ShotCalculator.HUB);
        this.inAllianceZoneDebounced = this.inAllianceZone.debounce(1.35);

        this.inNeutralZone = this.inAllianceZone.negate();
        this.inNeutralZoneDebounced = this.inNeutralZone.debounce(1.75);

        RobotModeTriggers.disabled().onTrue(this.runOnce(() -> this.state = LaunchState.AUTOMATIC).ignoringDisable(true));
    }

    public Trigger isLaunching() {
        return this.isLaunching;
    }

    public Command setState(LaunchState state) {
        return runOnce(() -> this.setLaunchState(state));
    }

    public void setLaunchState(LaunchState state) {
        this.state = state;
    }

    @Override
    public void periodic() {
        RobotContainer robot = RobotContainer.getInstance();
        Pose2d pose = robot.drive.getPose();

        this.shotCalculator = this.selectTarget(pose.getTranslation());

        ShotParameters params = this.shotCalculator.calculate(robot);
        boolean hoodManual = robot.operatorController.rightTrigger().getAsBoolean();
        boolean isTest = DriverStation.isTest();
        TestMode testMode = robot.getTestMode();
        
        this.autofire = this.shouldAutofire(pose.getTranslation(), params.timeOfFlight());
        boolean shouldLaunch = this.isLaunching.getAsBoolean();

        if (isTest && testMode == TestMode.TURRET) {
            Translation2d opManual = new Translation2d(
                robot.operatorController.getRightY(),
                -robot.operatorController.getRightX()
            );
            if (opManual.getNorm() > 0.5) this.turret.setAngleFieldRelative(opManual.getAngle().plus(Rotation2d.kCCW_90deg));
            if (!hoodManual) this.hood.setAngle(0);
            this.flywheel.setRPM(0);
        }
        else if (isTest && testMode == TestMode.LOOKUP_TABLES) {
            this.turret.setAngleFieldRelative(params.turretAngle());
            if (!hoodManual) this.hood.setAngle(this.manualHoodAngle);
            this.flywheel.setRPM(this.manualFlywheelRPM);

            Logger.recordOutput("ShotCalculator/ManualHoodAngle", this.manualHoodAngle);
            Logger.recordOutput("ShotCalculator/ManualFlywheelRPM", this.manualFlywheelRPM);
        }
        else if (!isTest) {
            this.turret.setAngleFieldRelative(params.turretAngle());
            if (!hoodManual) this.hood.setAngle(params.hoodAngle());
            if (shouldLaunch) this.flywheel.setRPM(params.flywheelRpm());
            else this.flywheel.setVoltage(0);
        }

        turret.periodic();
        hood.periodic();
        flywheel.periodic();
        
        Logger.recordOutput("Launcher/LaunchState", this.state);
        Logger.recordOutput("Launcher/Autofire", this.autofire);
        Logger.recordOutput("Launcher/IsLaunching", shouldLaunch);

        Logger.recordOutput("Launcher/Triggers/InAllianceZone", this.inAllianceZone.getAsBoolean());
        Logger.recordOutput("Launcher/Triggers/InAllianceZoneDB", this.inAllianceZoneDebounced.getAsBoolean());
        Logger.recordOutput("Launcher/Triggers/InNeutralZone", this.inNeutralZone.getAsBoolean());
        Logger.recordOutput("Launcher/Triggers/InNeutralZoneDB", this.inNeutralZoneDebounced.getAsBoolean());

        Utils.logActiveCommand("Launcher", this);
    }

    public boolean shouldAutofire(Translation2d robot, double timeOfFlight) {
        if (DriverStation.isAutonomous()) return false;

        // get flipped turret position
        Translation2d turretPos = turret.getFieldSpacePose().getTranslation();
        Translation2d flipped = AllianceFlipUtil.shouldFlip() ? AllianceFlipUtil.flip(turretPos) : turretPos;
        
        // should we try to score in the hub?
        if (this.inAllianceZone.getAsBoolean()) {
            if (!this.willFuelBeScored(timeOfFlight)) return false;
            return this.inAllianceZoneDebounced.getAsBoolean()
                && !FieldConstants.noFireZoneTower.contains(flipped);
        }

        // should we try to pass?
        return this.inNeutralZoneDebounced.getAsBoolean()
            && !FieldConstants.noFireZoneNet.contains(flipped);
    }

    public boolean willFuelBeScored(double timeOfFlight) {
        ShiftInfo shift = ShiftInfo.getCurrentShift();
        double endOffset = FieldConstants.hubDeactivationSeconds - timeOfFlight - FieldConstants.hubMaxProcessSeconds;
        if (shift.isHubActive()) {
            double timeUntilInactive = ShiftInfo.getTimeUntilInactive();
            if (endOffset < 0 && timeUntilInactive >= 0) return timeUntilInactive > -endOffset;
            return true;
        }
        if (endOffset > 0 && shift.timeSinceStart() < endOffset) return true;
        return ShiftInfo.getTimeUntilActive() < timeOfFlight
            + FieldConstants.hubMinProcessSeconds + Flywheel.Constants.SPIN_UP_TIME;
    }

    public ShotCalculator selectTarget(Translation2d robot) {
        if (AllianceFlipUtil.shouldFlip()) robot = new Translation2d(AllianceFlipUtil.flipX(robot.getX()), robot.getY());

        if (robot.getX() < FieldConstants.allianceZoneX) return ShotCalculator.HUB;
        return ShotCalculator.DYNAMIC_PASSING;
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
        if (AllianceFlipUtil.shouldFlip()) return priority ? FieldConstants.passingMidZoneY1 : FieldConstants.passingMidZoneY2;
        return priority ? FieldConstants.passingMidZoneY2 : FieldConstants.passingMidZoneY1;
    }

    public Optional<Boolean> getDefaultPassingPriority() {
        if (DriverStation.getRawAllianceStation().toString().contains("1")) return Optional.of(false);
        if (DriverStation.getRawAllianceStation().toString().contains("2")) return Optional.of(false);
        if (DriverStation.getRawAllianceStation().toString().contains("3")) return Optional.of(true);
        return Optional.empty();
    }

    public boolean shouldLimitDrive() {
        return this.isLaunching.getAsBoolean() && this.inAllianceZone.getAsBoolean();
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
