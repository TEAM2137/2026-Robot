package frc.robot.subsystems.launcher.turret;

import java.util.Map;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.RobotContainer;
import frc.robot.util.Alerts;
import frc.robot.util.Utils;

public class Turret {
    public static class Constants {
        public static final double cwBound = 230;
        public static final double cwMinBound = 210;
        public static final double ccwBound = 180;
        public static final double ccwMinBound = 160;

        public static final double offsetX = 6.375; // inches, positive towards robot right
        public static final double offsetY = -5.875; // inches, positive towards robot front

        public static final Translation2d turretOffset = new Translation2d(Constants.offsetY / 39.37, -Constants.offsetX / 39.37);

        public static final double magnetPosition = 0.004150390625; // rotations
        public static final double tempOffsetDegrees = -3;

        public static final InterpolatingDoubleTreeMap offsetLookup = InterpolatingDoubleTreeMap.ofEntries(
            // Map.entry(0.0, 0.0)

            Map.entry(-180.0, 4.0),
            Map.entry(-135.0, 3.0),
            Map.entry(-90.0, -1.0),
            Map.entry(-45.0, -5.0),
            Map.entry(0.0, -8.0),
            Map.entry(45.0, -6.0),
            Map.entry(90.0, -3.0),
            Map.entry(135.0, 0.0),
            Map.entry(180.0, 2.0)
        );
    }

    private final TurretIO io;
    private final TurretIOInputsAutoLogged inputs;

    private final Trigger isAtTargetTrigger;

    public double manualOffset = 0.0;
    public boolean didZero = false;

    public Turret(TurretIO io) {
        this.io = io;
        this.inputs = new TurretIOInputsAutoLogged();
        
        this.isAtTargetTrigger = new Trigger(io::isAtTarget);

        Alerts.add("Turret encoder not zeroed", AlertType.kWarning, () -> !this.didZero);
        Alerts.add("Turret motor disconnected", AlertType.kError, () -> !inputs.connected);
    }

    public Rotation2d getAngle() {
        return Rotation2d.fromDegrees(io.getAngle());
    }

    public Trigger isAtTarget() {
        return isAtTargetTrigger;
    }

    public void setAngleFieldRelative(Rotation2d angle) {
        Rotation2d robot = RobotContainer.getInstance().drive.getRotation();
        this.setAngleRobotRelative(angle.relativeTo(robot.unaryMinus()));
    }

    public void setAngleRobotRelative(Rotation2d angle) {
        if (!this.didZero && !Utils.isSim()) return;

        Rotation2d target = angle.unaryMinus().plus(Rotation2d.kCW_90deg)
            .plus(Rotation2d.fromDegrees(manualOffset));
        
        // offset the target based on a lookup table
        double offsetDegrees = Constants.offsetLookup.get(target.getDegrees()) + Constants.tempOffsetDegrees;
        // if (!SmartDashboard.containsKey("AimOffset")) SmartDashboard.putNumber("AimOffset", 0.0);
        // double offsetDegrees = SmartDashboard.getNumber("AimOffset", 0.0);
        target = target.plus(Rotation2d.fromDegrees(offsetDegrees));
        Logger.recordOutput("Launcher/Turret/AimOffsetDegrees", offsetDegrees);
        Logger.recordOutput("Launcher/Turret/AimOffsetInput", target.getDegrees());
        Logger.recordOutput("Launcher/Turret/TargetAngle", target);

        // use smaller bounds if the robot isn't launching fuel
        boolean isLaunching = RobotContainer.getInstance().launcher.isLaunching().getAsBoolean();
        double cwBound = isLaunching ? Constants.cwBound : Constants.cwMinBound;
        double ccwBound = isLaunching ? Constants.ccwBound : Constants.ccwMinBound;

        Rotation2d current = Rotation2d.fromDegrees(io.getAngle());
        double difference = target.getDegrees() - current.getDegrees();
        double output = target.getDegrees();
        if (difference > 180 && (target.getDegrees() - 360) > -cwBound) {
            output -= 360;
            Logger.recordOutput("Launcher/Turret/AdjustedTarget", target.getDegrees() - 360);
        }
        else if (difference < -180 && (target.getDegrees() + 360) < ccwBound) {
            output += 360;
            Logger.recordOutput("Launcher/Turret/AdjustedTarget", target.getDegrees() + 360);
        }
        else Logger.recordOutput("Launcher/Turret/AdjustedTarget", target.getDegrees());

        Logger.recordOutput("Launcher/Turret/Difference", difference);
        Logger.recordOutput("Launcher/Turret/Output", output);

        double robotRadsPerSec = RobotContainer.getInstance().drive.getAngularVelocityRadsPerSec();
        io.setAngleAndVelocity(output / 360.0, -robotRadsPerSec / (2 * Math.PI));
    }

    public Command increaseTurretOffset() {
        return Commands.runOnce(() -> manualOffset += 1);
    }

    public Command decreaseTurretOffset() {
        return Commands.runOnce(() -> manualOffset -= 1);
    }

    public Command resetTurretOffset() {
        return Commands.runOnce(() -> manualOffset = 0);
    }

    public Command resetPosition() {
        return Commands.runOnce(() -> io.setPosition(0.0));
    }

    public Command markAsUnzeroed() {
        return Commands.runOnce(() -> this.didZero = false);
    }

    public Command setVoltage(double volts) {
        return Commands.runOnce(() -> {
            if (!this.didZero) this.io.setVoltage(volts);
        });
    }

    public Trigger isZeroed() {
        return new Trigger(() -> this.didZero);
    }

    public Trigger isNotZeroed() {
        return new Trigger(() -> !this.didZero);
    }

    public Pose2d getFieldSpacePose() {
        RobotContainer robot = RobotContainer.getInstance();
        Pose2d robotPose = robot.drive.getPose();
        Transform2d transform = new Transform2d(
            Constants.turretOffset,
            robotPose.getRotation()
        );
        return new Pose2d(
            robotPose.transformBy(transform).getTranslation(),
            this.getAngle().plus(robot.drive.getRotation())
        );
    }

    public Translation2d getFieldSpaceVelocity() {
        RobotContainer robot = RobotContainer.getInstance();
        Pose2d robotPose = robot.drive.getPose();
        double omega = robot.drive.getAngularVelocityRadsPerSec();
        Translation2d world = Constants.turretOffset.rotateBy(robotPose.getRotation());
        Translation2d turretVelocity = new Translation2d(
            -omega * world.getY(),
            omega * world.getX()
        );
        return robot.drive.getLinearSpeedsVector().plus(turretVelocity);
    }

    public void periodic() {
        boolean previousSensorValue = inputs.sensorValue;

        io.updateInputs(inputs);
        Logger.processInputs("Launcher/Turret", inputs);

        if (!this.didZero && inputs.sensorValue && !previousSensorValue && inputs.velocityRotationsPerSecond < 0) {
            io.setPosition(-Constants.magnetPosition);
            this.didZero = true;
        }

        Logger.recordOutput("Launcher/Turret/DidZero", this.didZero);
        Logger.recordOutput("Launcher/Turret/ManualOffset", this.manualOffset);
        Logger.recordOutput("Launcher/Turret/FieldSpacePose", this.getFieldSpacePose());
        Logger.recordOutput("Launcher/Turret/FieldSpaceVelocity", this.getFieldSpaceVelocity());
    }
}
