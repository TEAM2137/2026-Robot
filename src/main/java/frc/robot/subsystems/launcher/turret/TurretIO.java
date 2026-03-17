package frc.robot.subsystems.launcher.turret;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;

public interface TurretIO {
    @AutoLog
    public static class TurretIOInputs {
        public double appliedVolts;
        public double statorCurrentAmps;
        public double supplyCurrentAmps;
        public double motorTempCelsius;

        public double angleRotations;
        public double targetAngleRotations;
        public double velocityRotationsPerSecond;
        
        public Rotation2d angle;

        public boolean sensorValue;
        public boolean connected;
    }

    default void setAngle(double degrees) {}
    default void setAngleAndVelocity(double degrees, double velocity) {}
    default double getAngle() { return 0.0; }
    default boolean isAtTarget() { return true; }
    default void setPosition(double position) {}
    default void setVoltage(double volts) {} 

    default void updateInputs(TurretIOInputs inputs) {}
}
