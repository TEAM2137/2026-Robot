package frc.robot.subsystems.launcher;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.RobotContainer;

public interface ShotCalculator {
    ShotParameters calculate(RobotContainer robot);

    public record ShotParameters(Rotation2d turretAngle, double flywheelRpm, double hoodAngle) {}
}
