package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.RPM;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class IntakeIOTalonFX implements IntakeIO {
    public static class Constants {
        public static final int pivotId = 40;
        public static final int rollerLeaderId = 41;
        public static final int rollerFollowerId = 42;
        public static final double pivotGearing = 3753.0 / 1568.0;

        public static final double kP = 16.0;
        public static final double kD = 0.1;
        public static final double holdVoltage = 0.1;

        public static final double rollerKP = 0.15;//0.25;
        public static final double rollerKV = 0.117;//0.19;

        public static final double cruiseVelocity = 16;
        public static final double acceleration = 42;
    }

    protected final TalonFX pivot;
    protected final TalonFX leader;
    protected final TalonFX follower;

    protected double targetPositionRotations;
    protected double rollerTargetVelocity;

    public IntakeIOTalonFX() {
        // roller configs

        this.leader = new TalonFX(Constants.rollerLeaderId);
        this.leader.getConfigurator().apply(new MotorOutputConfigs()
            .withNeutralMode(NeutralModeValue.Brake)
            .withInverted(InvertedValue.Clockwise_Positive));

        this.leader.getConfigurator().apply(new Slot0Configs()
            .withKP(Constants.rollerKP).withKV(Constants.rollerKV));
        
        this.leader.getConfigurator().apply(new CurrentLimitsConfigs()
            .withSupplyCurrentLimit(25)
            .withSupplyCurrentLimitEnable(true));

        this.follower = new TalonFX(Constants.rollerFollowerId);
        this.follower.getConfigurator().apply(new MotorOutputConfigs()
            .withNeutralMode(NeutralModeValue.Brake)
            .withInverted(InvertedValue.Clockwise_Positive));
        
        this.follower.getConfigurator().apply(new CurrentLimitsConfigs()
            .withSupplyCurrentLimit(25)
            .withSupplyCurrentLimitEnable(true));

        this.follower.setControl(new Follower(this.leader.getDeviceID(), MotorAlignmentValue.Opposed));

        // pivot configs

        this.pivot = new TalonFX(Constants.pivotId);
        this.pivot.getConfigurator().apply(new FeedbackConfigs()
            .withSensorToMechanismRatio(Constants.pivotGearing));

        this.pivot.getConfigurator().apply(new Slot0Configs()
            .withKP(Constants.kP).withKD(Constants.kD));
        this.pivot.getConfigurator().apply(new Slot1Configs()
            .withKP(Constants.kP).withKD(Constants.kD)
            .withKG(Constants.holdVoltage));

        this.pivot.getConfigurator().apply(new MotionMagicConfigs()
            .withMotionMagicCruiseVelocity(Constants.cruiseVelocity)
            .withMotionMagicAcceleration(Constants.acceleration));
        
        this.pivot.getConfigurator().apply(new CurrentLimitsConfigs()
            .withSupplyCurrentLimit(25)
            .withSupplyCurrentLimitEnable(true));

        // reset encoder on startup
        this.resetPosition();
    }

    @Override
    public void setRollerVoltage(double volts) {
        this.leader.setVoltage(volts);
    }

    @Override
    public void setRollerRPM(double rpm) {
        this.rollerTargetVelocity = rpm;
        this.leader.setControl(new VelocityVoltage(rpm / 60.0));
    }

    @Override
    public void setPosition(double position) {
        this.targetPositionRotations = position;
        this.pivot.setControl(new MotionMagicVoltage(position));
    }

    @Override
    public void setPositionAndHold(double position) {
        this.targetPositionRotations = position;
        this.pivot.setControl(new MotionMagicVoltage(position).withSlot(1));
    }

    @Override
    public void setPivotVoltage(double volts) {
        this.targetPositionRotations = 0.0;
        this.pivot.setVoltage(volts);
    }

    @Override
    public void resetPosition() {
        this.pivot.setPosition(0.0);
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        inputs.pivotPositionRotations = this.pivot.getPosition().getValueAsDouble();
        inputs.pivotTargetPositionRotations = this.targetPositionRotations;

        inputs.pivotAppliedVolts = this.pivot.getMotorVoltage().getValueAsDouble();
        inputs.pivotStatorCurrentAmps = this.pivot.getStatorCurrent().getValueAsDouble();
        inputs.pivotSupplyCurrentAmps = this.pivot.getSupplyCurrent().getValueAsDouble();
        inputs.pivotTempCelsius = this.pivot.getDeviceTemp().getValueAsDouble();
        inputs.pivotConnected = this.pivot.isConnected();

        inputs.rollerAppliedVolts = this.leader.getMotorVoltage().getValueAsDouble();
        inputs.rollerStatorCurrentAmps = this.leader.getStatorCurrent().getValueAsDouble();
        inputs.rollerSupplyCurrentAmps = this.leader.getSupplyCurrent().getValueAsDouble();
        inputs.rollerTempCelsius = this.leader.getDeviceTemp().getValueAsDouble();
        inputs.rollerVelocityRpm = this.leader.getVelocity().getValue().in(RPM);
        inputs.rollerTargetVelocityRpm = this.rollerTargetVelocity;
        inputs.rollersConnected = this.leader.isConnected();
    }
}