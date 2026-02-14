package frc.robot.subsystems.launcher.turret;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.RobotContainer;
import frc.robot.util.Alerts;

public class Turret {
    public static class Constants {
        public static final double overlapDegrees = 50;
        public static final double leftBound = -180 - overlapDegrees / 2.0;
        public static final double rightBound = 180 + overlapDegrees / 2.0;

        public static final double offsetX = 6.375; // inches, positive towards robot right
        public static final double offsetY = -5.875; // inches, positive towards robot front
    }

    public double turretOffset = 0.0;

    private final TurretIO io;
    private final TurretIOInputsAutoLogged inputs;

    private final Trigger isAtTargetTrigger;

    public Turret(TurretIO io) {
        this.io = io;
        this.inputs = new TurretIOInputsAutoLogged();
        
        this.isAtTargetTrigger = new Trigger(io::isAtTarget);

        Alerts.add("Turret encoder not zeroed", AlertType.kWarning, () -> !inputs.didZero);
        Alerts.add("Turret motor disconnected", AlertType.kError, () -> !inputs.connected);
    }

    public void setAngleFieldRelative(Rotation2d angle) {
        Rotation2d robot = RobotContainer.getInstance().drive.getRotation();
        this.setAngleRobotRelative(angle.relativeTo(robot.unaryMinus()));
    }

    public void setAngleRobotRelative(Rotation2d angle) {
        Rotation2d target = angle.unaryMinus().plus(Rotation2d.kCW_90deg)
            .plus(Rotation2d.fromDegrees(turretOffset));
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

        double robotRadsPerSec = RobotContainer.getInstance().drive.getAngularSpeedRadsPerSec();
        io.setAngleAndVelocity(output / 360.0, -robotRadsPerSec / (2 * Math.PI));
    }

    public Rotation2d getAngle() {
        return Rotation2d.fromDegrees(io.getAngle());
    }

    public Trigger isAtTarget() {
        return isAtTargetTrigger;
    }

    public Command increaseTurretOffset() {
        return Commands.run(()-> turretOffset += 0.5);
    }

    public Command decreaseTurretOffset() {
        return Commands.run(()-> turretOffset -= 0.5);
    }

    public Command resetTurretOffset() {
        return Commands.runOnce(()-> turretOffset = 0);
    }

    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Launcher/Turret", inputs);
        Logger.recordOutput("Launcher/TurretOffset", turretOffset);
        Logger.recordOutput("Launcher/FieldSpacePose", this.getFieldSpacePose(RobotContainer.getInstance()));
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
}
