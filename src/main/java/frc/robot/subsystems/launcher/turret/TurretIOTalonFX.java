package frc.robot.subsystems.launcher.turret;

import static edu.wpi.first.units.Units.Degrees;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.math.geometry.Rotation2d;

public class TurretIOTalonFX implements TurretIO {
    public static class Constants {
        public static final double gearing = 39168.0 / 847.0;
    }

    protected final TalonFX motor;

    public TurretIOTalonFX() {
        motor = new TalonFX(50);

        motor.getConfigurator().apply(new MotorOutputConfigs()
            .withInverted(InvertedValue.Clockwise_Positive));

        motor.getConfigurator().apply(new FeedbackConfigs()
            .withSensorToMechanismRatio(Constants.gearing));

        motor.getConfigurator().apply(new Slot0Configs()
            .withKP(5).withKD(1.5));

        motor.getConfigurator().apply(new MotionMagicConfigs()
            .withMotionMagicCruiseVelocity(5)
            .withMotionMagicAcceleration(15));
    }

    @Override
    public double getAngle() {
        return this.motor.getPosition().getValue().in(Degrees);
    }

    @Override
    public void setAngle(double degrees) {
        this.motor.setControl(new MotionMagicVoltage(degrees / 360.0));
    }

    @Override
    public boolean isAtTarget() {
        return Math.abs(this.motor.getClosedLoopError().getValueAsDouble()) < 0.025;
    }

    @Override
    public void updateInputs(TurretIOInputs inputs) {
        inputs.angleRaw = this.motor.getPosition().getValueAsDouble();
        inputs.angle = Rotation2d.fromRotations(inputs.angleRaw);
        inputs.didZero = false;
        inputs.connected = this.motor.isConnected();
    }
}
