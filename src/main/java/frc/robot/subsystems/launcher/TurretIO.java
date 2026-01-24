package frc.robot.subsystems.launcher;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;

public interface TurretIO {
    @AutoLog
    public static class TurretIOInputs {
        public double angleRaw;
        public Rotation2d angleRobotRelative = new Rotation2d();
        public Rotation2d angleFieldRelative = new Rotation2d();
        public boolean didZero;
    }

    default void setAngleRaw(double angle) {}
    default void setAngleRobotRelative(Rotation2d angle) {}
    default void setAngleFieldRelative(Rotation2d angle) {}
    default void setShotCalculator(ShotCalculator calculator) {}

    default void updateInputs(TurretIOInputs inputs) {}
}
