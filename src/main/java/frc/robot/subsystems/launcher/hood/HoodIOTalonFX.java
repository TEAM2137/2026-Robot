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
        public static final double gearing = 43;

        public static final double kP = 8.0;
        public static final double kD = 0.1;
    }

    protected final TalonFX motor;

    public HoodIOTalonFX() {
        this.motor = new TalonFX(35);

        this.motor.getConfigurator().apply(new MotorOutputConfigs()
            .withInverted(InvertedValue.CounterClockwise_Positive));

        this.motor.getConfigurator().apply(new FeedbackConfigs()
            .withSensorToMechanismRatio(Constants.gearing));

        this.motor.getConfigurator().apply(new Slot0Configs()
            .withKP(Constants.kP).withKD(Constants.kD));
    }

    @Override
    public void setAngle(double degrees) {
        this.motor.setControl(new PositionVoltage(degrees / 360.0));
    }

    @Override
    public void updateInputs(HoodIOInputs inputs) {
        inputs.angleDegrees = this.motor.getPosition().getValue().in(Degrees);
        inputs.connected = this.motor.isConnected();
    }
}
