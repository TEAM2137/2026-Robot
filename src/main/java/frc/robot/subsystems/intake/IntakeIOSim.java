package frc.robot.subsystems.intake;

import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class IntakeIOSim extends IntakeIOTalonFX {
    private TalonFXSimState rollerSimState;
    private DCMotorSim rollerSim = new DCMotorSim(
        LinearSystemId.createDCMotorSystem(
            DCMotor.getKrakenX60(1),
            0.01,
            1.0
        ),
        DCMotor.getKrakenX60(1)
    );

    public IntakeIOSim() {
        this.rollerSimState = this.rollers.getSimState();
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        rollerSimState.setSupplyVoltage(RobotController.getBatteryVoltage());

        rollerSim.setInputVoltage(rollerSimState.getMotorVoltage());
        rollerSim.update(0.02);

        rollerSimState.setRotorVelocity(rollerSim.getAngularVelocity());
        
        super.updateInputs(inputs);
    }
}
