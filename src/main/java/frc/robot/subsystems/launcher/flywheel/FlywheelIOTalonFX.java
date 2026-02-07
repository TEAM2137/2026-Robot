package frc.robot.subsystems.launcher.flywheel;

import static edu.wpi.first.units.Units.RPM;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

public class FlywheelIOTalonFX implements FlywheelIO {
    public static class Constants {
        public static final int leaderId = 30;
        public static final int followerId = 31;
        public static final int feederId = 32;
        public static final double gearing = 11.0 / 12.0;
    }

    protected final TalonFX leader;
    protected final TalonFX follower;
    protected final TalonFX feeder;

    public FlywheelIOTalonFX() {
        this.leader = new TalonFX(Constants.leaderId);
        this.leader.getConfigurator().apply(new Slot0Configs()
            .withKP(0.1).withKS(0.12).withKV(0.101));

        this.follower = new TalonFX(Constants.followerId);
        this.feeder = new TalonFX(Constants.feederId);
        
        this.follower.setControl(new Follower(this.leader.getDeviceID(), MotorAlignmentValue.Opposed));
        this.feeder.setControl(new Follower(this.leader.getDeviceID(), MotorAlignmentValue.Opposed));
    }

    @Override
    public void setVoltage(double volts) {
        this.leader.setVoltage(volts);
    }

    @Override
    public void setRPM(double rpm) {
        this.leader.setControl(new VelocityVoltage(rpm / 60.0));
    }

    @Override
    public void updateInputs(FlywheelIOInputs inputs) {
        inputs.speedRpm = this.leader.getVelocity().getValue().in(RPM);
        inputs.flywheelConnected = this.leader.isConnected() && this.follower.isConnected();
        inputs.feederConnected = this.feeder.isConnected();
    }
}
