package frc.robot.subsystems.launcher.turret;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.RobotContainer;

public class Turret {
    public static class Constants {
        public static final double overlapDegrees = 50;
        public static final double leftBound = -180 - overlapDegrees / 2.0;
        public static final double rightBound = 180 + overlapDegrees / 2.0;
    }

    private final TurretIO io;
    private final TurretIOInputsAutoLogged inputs;

    private final Trigger isAtTargetTrigger;

    public Turret(TurretIO io) {
        this.io = io;
        this.inputs = new TurretIOInputsAutoLogged();
        
        this.isAtTargetTrigger = new Trigger(io::isAtTarget);
    }

    public void setAngleFieldRelative(Rotation2d angle) {
        Rotation2d robot = RobotContainer.getInstance().drive.getRotation();
        this.setAngleRobotRelative(angle.relativeTo(robot.unaryMinus()));
    }

    public void setAngleRobotRelative(Rotation2d angle) {
        Rotation2d target = angle.unaryMinus().plus(Rotation2d.kCW_90deg);
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

        io.setAngle(output);
    }

    public Rotation2d getAngle() {
        return Rotation2d.fromDegrees(io.getAngle());
    }

    public Trigger isAtTarget() {
        return isAtTargetTrigger;
    }

    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Launcher/Turret", inputs);
    }
}
