package frc.robot.subsystems.launcher.turret;

import static edu.wpi.first.units.Units.Degrees;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DigitalInput;

public class TurretIOTalonFX implements TurretIO {
    public static class Constants {
        public static final int id = 50;
        
        public static final double gearing = 4352.0 / 77.0;

        public static final double kP = 80.0;
        public static final double kD = 1.5;
        public static final double kV = 8.0; //8 in sim, 5.52 calculated
        public static final double kS = 0.22; 
    }

    protected final TalonFX motor;
    protected final DigitalInput sensor;

    private double target = 0;

    public TurretIOTalonFX() {
        this.motor = new TalonFX(Constants.id, "turret");

        this.motor.getConfigurator().apply(new MotorOutputConfigs()
            .withInverted(InvertedValue.CounterClockwise_Positive));

        this.motor.getConfigurator().apply(new FeedbackConfigs()
            .withSensorToMechanismRatio(Constants.gearing));

        this.motor.getConfigurator().apply(new Slot0Configs()
            .withKP(Constants.kP).withKD(Constants.kD)
            .withKV(Constants.kV).withKS(Constants.kS)
            .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign));

        this.resetPosition();

        this.sensor = new DigitalInput(9);
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
    public void resetPosition() {
        this.motor.setPosition(0.0);
    }

    @Override
    public void updateInputs(TurretIOInputs inputs) {
        inputs.angleRotations = this.motor.getPosition().getValueAsDouble();
        inputs.angle = Rotation2d.fromRotations(inputs.angleRotations);
        inputs.targetAngleRotations = this.target;
        inputs.velocityRotationsPerSecond = this.motor.getVelocity().getValueAsDouble();
        inputs.appliedVolts = this.motor.getMotorVoltage().getValueAsDouble();

        inputs.sensorValue = !this.sensor.get();
        inputs.connected = this.motor.isConnected();
    }
}
