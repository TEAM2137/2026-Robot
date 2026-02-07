package frc.robot.subsystems.launcher.hood;

import static edu.wpi.first.units.Units.Degrees;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

public class HoodIOTalonFX implements HoodIO {
    protected final TalonFX motor;

    public HoodIOTalonFX() {
        this.motor = new TalonFX(35);
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
