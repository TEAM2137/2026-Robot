package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.RPM;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class IntakeIOTalonFX implements IntakeIO {
    public static class Constants {
        public static final int rollersId = 40;
        public static final int pivotId = 41;
        public static final double pivotGearing = 3753.0 / 1568.0;

        public static final double kP = 16.0;
        public static final double kD = 0.1;

        public static final double rollerKP = 0.15;//0.25;
        public static final double rollerKV = 0.117;//0.19;

        public static final double cruiseVelocity = 16;
        public static final double acceleration = 50;
    }

    protected final TalonFX pivot;
    protected final TalonFX rollers;

    protected double targetPositionRotations;
    protected double rollerTargetVelocity;

    public IntakeIOTalonFX() {
        // roller configs

        this.rollers = new TalonFX(Constants.rollersId);
        this.rollers.getConfigurator().apply(new MotorOutputConfigs()
            .withNeutralMode(NeutralModeValue.Brake)
            .withInverted(InvertedValue.CounterClockwise_Positive));

        this.rollers.getConfigurator().apply(new Slot0Configs()
            .withKP(Constants.rollerKP).withKV(Constants.rollerKV));
        
        this.rollers.getConfigurator().apply(new CurrentLimitsConfigs()
            .withSupplyCurrentLimit(50)
            .withSupplyCurrentLimitEnable(true));

        // pivot configs

        this.pivot = new TalonFX(Constants.pivotId);
        this.pivot.getConfigurator().apply(new FeedbackConfigs()
            .withSensorToMechanismRatio(Constants.pivotGearing));

        this.pivot.getConfigurator().apply(new Slot0Configs()
            .withKP(Constants.kP).withKD(Constants.kD));

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
        this.rollers.setVoltage(volts);
    }

    @Override
    public void setRollerRPM(double rpm) {
        this.rollerTargetVelocity = rpm;
        this.rollers.setControl(new VelocityVoltage(rpm / 60.0));
    }

    @Override
    public void setPosition(double position) {
        this.targetPositionRotations = position;
        this.pivot.setControl(new MotionMagicVoltage(position));
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

        inputs.rollerAppliedVolts = this.rollers.getMotorVoltage().getValueAsDouble();
        inputs.rollerStatorCurrentAmps = this.rollers.getStatorCurrent().getValueAsDouble();
        inputs.rollerSupplyCurrentAmps = this.rollers.getSupplyCurrent().getValueAsDouble();
        inputs.rollerTempCelsius = this.rollers.getDeviceTemp().getValueAsDouble();
        inputs.rollerVelocityRpm = this.rollers.getVelocity().getValue().in(RPM);
        inputs.rollerTargetVelocityRpm = this.rollerTargetVelocity;
        inputs.rollersConnected = this.rollers.isConnected();
    }
}