package frc.robot.subsystems.launcher.turret;

import java.util.Map;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
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
        public static final double leftBound = -205;
        public static final double rightBound = 205;

        public static final double offsetX = 6.375; // inches, positive towards robot right
        public static final double offsetY = -5.875; // inches, positive towards robot front

        public static final double magnetPosition = 0.004150390625; // rotations
        
        public static final double baseOffset = 7.5; // degrees
        public static final InterpolatingDoubleTreeMap offsetLookup = InterpolatingDoubleTreeMap.ofEntries(
            Map.entry(-315.0, 0.0),
            Map.entry(-135.0, 1.0),
            Map.entry(45.0, 0.0),
            Map.entry(225.0, 1.0)
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
        
        double offsetDegrees = Constants.baseOffset * Constants.offsetLookup.get(target.getDegrees());
        Logger.recordOutput("Launcher/Turret/AimOffsetDegrees", offsetDegrees);
        target = target.plus(Rotation2d.fromDegrees(offsetDegrees));

        Rotation2d current = Rotation2d.fromDegrees(io.getAngle());

        Logger.recordOutput("Launcher/Turret/TargetAngle", target);

        double difference = target.getDegrees() - current.getDegrees();
        double output = target.getDegrees();
        if (difference > 180 && (target.getDegrees() - 360) > Constants.leftBound) {
            output -= 360;
            Logger.recordOutput("Launcher/Turret/AdjustedTarget", target.getDegrees() - 360);
        }
        else if (difference < -180 && (target.getDegrees() + 360) < Constants.rightBound) {
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
        return Commands.run(() -> manualOffset += 0.5);
    }

    public Command decreaseTurretOffset() {
        return Commands.run(() -> manualOffset -= 0.5);
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

    public Pose2d getFieldSpacePose(RobotContainer robot) {
        Pose2d robotPose = robot.drive.getPose();
        Transform2d transform = new Transform2d(
            Constants.offsetY / 39.37,
            -Constants.offsetX / 39.37,
            robotPose.getRotation()
        );
        return new Pose2d(
            robotPose.transformBy(transform).getTranslation(),
            this.getAngle().plus(robot.drive.getRotation())
        );
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
        Logger.recordOutput("Launcher/Turret/FieldSpacePose", this.getFieldSpacePose(RobotContainer.getInstance()));
    }
}
