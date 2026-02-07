package frc.robot.subsystems.launcher.turret;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;

public interface TurretIO {
    @AutoLog
    public static class TurretIOInputs {
        public double angleRaw;
        public Rotation2d angle;
        public boolean didZero;
        public boolean connected;
    }

    default void setAngle(double degrees) {}
    default double getAngle() { return 0.0; }
    default boolean isAtTarget() { return true; }

    default void updateInputs(TurretIOInputs inputs) {}
}
