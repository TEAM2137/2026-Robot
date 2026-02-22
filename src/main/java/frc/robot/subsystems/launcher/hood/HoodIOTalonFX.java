package frc.robot.subsystems.launcher.hood;

import static edu.wpi.first.units.Units.Degrees;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

public class HoodIOTalonFX implements HoodIO {
    public static class Constants {
        public static final int id = 35;
        
        public static final double gearing = 43;

        public static final double kP = 125.0;
        public static final double kI = 300.0;
        public static final double kD = 0.0;
    }

    protected final TalonFX motor;

    protected double targetAngleDegrees;

    public HoodIOTalonFX() {
        this.motor = new TalonFX(Constants.id, "turret");

        this.motor.getConfigurator().apply(new MotorOutputConfigs()
            .withInverted(InvertedValue.CounterClockwise_Positive));

        this.motor.getConfigurator().apply(new FeedbackConfigs()
            .withSensorToMechanismRatio(Constants.gearing));

        this.motor.getConfigurator().apply(new Slot0Configs()
            .withKP(Constants.kP).withKI(Constants.kI).withKD(Constants.kD));

        this.motor.setPosition(0.0);
    }

    @Override
    public void setAngle(double degrees) {
        this.targetAngleDegrees = degrees;
        this.motor.setControl(new PositionVoltage(degrees / 360.0));
    }

    @Override
    public void updateInputs(HoodIOInputs inputs) {
        inputs.angleDegrees = this.motor.getPosition().getValue().in(Degrees);
        inputs.targetAngleDegrees = this.targetAngleDegrees;
        inputs.connected = this.motor.isConnected();
    }
}
