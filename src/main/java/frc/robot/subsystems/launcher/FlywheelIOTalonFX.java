package frc.robot.subsystems.launcher;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

public class FlywheelIOTalonFX implements FlywheelIO {
    private final TalonFX leader;
    private final TalonFX follower;
    private final TalonFX feeder;

    public FlywheelIOTalonFX() {
        this.leader = new TalonFX(30);
        this.follower = new TalonFX(31);
        this.feeder = new TalonFX(32);
        this.feeder.getConfigurator().apply(new MotorOutputConfigs()
            .withInverted(InvertedValue.Clockwise_Positive));
        
        this.follower.setControl(new Follower(this.leader.getDeviceID(), MotorAlignmentValue.Opposed));
    }

    @Override
    public void setVoltage(double volts) {
        this.leader.setVoltage(volts);
        this.feeder.setVoltage(volts);
    }

    @Override
    public void updateInputs(FlywheelIOInputs inputs) {
        inputs.speedRpm = this.leader.getVelocity().getValue().in(RotationsPerSecond) * 60;
    }
}
