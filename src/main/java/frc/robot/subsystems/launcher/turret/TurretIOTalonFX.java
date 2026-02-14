package frc.robot.subsystems.launcher.turret;

import static edu.wpi.first.units.Units.Degrees;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.math.geometry.Rotation2d;

public class TurretIOTalonFX implements TurretIO {
    public static class Constants {
        public static final double gearing = 39168.0 / 847.0;
    }

    protected final TalonFX motor;

    private double target = 0;

    public TurretIOTalonFX() {
        motor = new TalonFX(50);

        motor.getConfigurator().apply(new MotorOutputConfigs()
            .withInverted(InvertedValue.Clockwise_Positive));

        motor.getConfigurator().apply(new FeedbackConfigs()
            .withSensorToMechanismRatio(Constants.gearing));

        motor.getConfigurator().apply(new Slot0Configs()
            .withKP(80).withKD(1.5).withKV(8)); //5.52
    }

    @Override
    public double getAngle() {
        return this.motor.getPosition().getValue().in(Degrees);
    }

    @Override
    public void setAngle(double position) {
        this.target = position;
        this.motor.setControl(new PositionVoltage(position));
    }

    @Override
    public void setAngleAndVelocity(double position, double velocity) {
        this.target = position;
        this.motor.setControl(new PositionVoltage(position).withVelocity(velocity));
    }

    @Override
    public boolean isAtTarget() {
        return Math.abs(this.motor.getClosedLoopError().getValueAsDouble()) < 0.025;
    }

    @Override
    public void updateInputs(TurretIOInputs inputs) {
        inputs.angleRotations = this.motor.getPosition().getValueAsDouble();
        inputs.targetAngleRotations = this.target;
        inputs.velocityRotationsPerSecond = this.motor.getVelocity().getValueAsDouble();
        inputs.angle = Rotation2d.fromRotations(inputs.angleRotations);
        inputs.didZero = false;
        inputs.connected = this.motor.isConnected();
    }
}
