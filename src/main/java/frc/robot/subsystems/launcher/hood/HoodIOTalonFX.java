package frc.robot.subsystems.launcher.hood;

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
}
