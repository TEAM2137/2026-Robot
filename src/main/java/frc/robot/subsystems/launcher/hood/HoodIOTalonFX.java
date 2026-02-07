package frc.robot.subsystems.launcher.hood;

import static edu.wpi.first.units.Units.Degrees;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

public class HoodIOTalonFX implements HoodIO {
    public static class Constants {
        public static final double gearing = 1.0;
    }

    protected final TalonFX motor;

    public HoodIOTalonFX() {
        this.motor = new TalonFX(35);
        this.motor.getConfigurator().apply(new FeedbackConfigs()
            .withSensorToMechanismRatio(Constants.gearing));
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
