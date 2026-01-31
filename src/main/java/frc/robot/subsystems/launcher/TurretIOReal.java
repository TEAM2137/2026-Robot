package frc.robot.subsystems.launcher;

import static edu.wpi.first.units.Units.Degrees;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.math.geometry.Rotation2d;

public class TurretIOReal implements TurretIO {
    protected final TalonFX motor;

    public TurretIOReal() {
        motor = new TalonFX(50);

        motor.getConfigurator().apply(new Slot0Configs().withKP(5).withKD(1));
        motor.getConfigurator().apply(new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive));
    }

    @Override
    public double getAngle() {
        return motor.getPosition().getValue().in(Degrees);
    }

    @Override
    public void setAngle(double degrees) {
        motor.setControl(new PositionVoltage(degrees / 360.0));
    }

    @Override
    public void updateInputs(TurretIOInputs inputs) {
        inputs.angleRaw = motor.getPosition().getValueAsDouble();
        inputs.angle = Rotation2d.fromRotations(inputs.angleRaw);
        inputs.didZero = false;
    }
}
