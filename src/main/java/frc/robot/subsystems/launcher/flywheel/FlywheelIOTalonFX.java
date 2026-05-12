package frc.robot.subsystems.launcher.flywheel;

import static edu.wpi.first.units.Units.RPM;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class FlywheelIOTalonFX implements FlywheelIO {
    public static class Constants {
        public static final int leaderId = 30;
        public static final int followerId = 31;
        public static final double gearing = 23.0 / 22.0;

        public static final double kP = 0.1;
        public static final double kS = 0.36;
        public static final double kV = 0.116;

        public static final double supplyCurrentLimit = 50;
    }

    protected final TalonFX leader;
    protected final TalonFX follower;

    private double targetVelocityRpm = 0.0;

    public FlywheelIOTalonFX() {
        this.leader = new TalonFX(Constants.leaderId, "turret");
        this.leader.getConfigurator().apply(new MotorOutputConfigs()
            .withNeutralMode(NeutralModeValue.Coast));
        this.leader.getConfigurator().apply(new Slot0Configs()
            .withKP(Constants.kP).withKS(Constants.kS).withKV(Constants.kV));
        this.leader.getConfigurator().apply(new CurrentLimitsConfigs()
            .withSupplyCurrentLimit(Constants.supplyCurrentLimit)
            .withSupplyCurrentLimitEnable(true));
        this.leader.getConfigurator().apply(new FeedbackConfigs()
            .withSensorToMechanismRatio(Constants.gearing));

        this.follower = new TalonFX(Constants.followerId, "turret");
        this.follower.getConfigurator().apply(new MotorOutputConfigs()
            .withNeutralMode(NeutralModeValue.Coast));
        this.follower.getConfigurator().apply(new Slot0Configs()
            .withKP(Constants.kP).withKS(Constants.kS).withKV(Constants.kV));
        this.follower.getConfigurator().apply(new CurrentLimitsConfigs()
            .withSupplyCurrentLimit(Constants.supplyCurrentLimit)
            .withSupplyCurrentLimitEnable(true));
        this.follower.getConfigurator().apply(new FeedbackConfigs()
            .withSensorToMechanismRatio(Constants.gearing));
        
        this.follower.setControl(new Follower(this.leader.getDeviceID(), MotorAlignmentValue.Opposed));
    }

    @Override
    public void setVoltage(double volts) {
        this.targetVelocityRpm = 0.0;
        this.leader.setVoltage(volts);
    }

    @Override
    public void setRPM(double rpm) {
        this.targetVelocityRpm = rpm;
        this.leader.setControl(new VelocityVoltage(rpm / 60.0));
    }

    @Override
    public boolean isWithinTarget(double range) {
        return Math.abs(this.leader.getVelocity().getValue().in(RPM) - this.targetVelocityRpm) < range;
    }

    @Override
    public void updateInputs(FlywheelIOInputs inputs) {
        inputs.leaderVelocityRpm = this.leader.getVelocity().getValue().in(RPM);
        inputs.leaderAppliedVolts = this.leader.getMotorVoltage().getValueAsDouble();
        inputs.leaderStatorCurrentAmps = this.leader.getStatorCurrent().getValueAsDouble();
        inputs.leaderSupplyCurrentAmps = this.leader.getSupplyCurrent().getValueAsDouble();
        inputs.leaderMotorTempCelsius = this.leader.getDeviceTemp().getValueAsDouble();
        inputs.leaderConnected = this.leader.isConnected();

        inputs.followerVelocityRpm = this.follower.getVelocity().getValue().in(RPM);
        inputs.followerAppliedVolts = this.follower.getMotorVoltage().getValueAsDouble();
        inputs.followerStatorCurrentAmps = this.follower.getStatorCurrent().getValueAsDouble();
        inputs.followerSupplyCurrentAmps = this.follower.getSupplyCurrent().getValueAsDouble();
        inputs.followerMotorTempCelsius = this.follower.getDeviceTemp().getValueAsDouble();
        inputs.followerConnected = this.follower.isConnected();

        inputs.targetVelocityRpm = this.targetVelocityRpm;
    }
}
