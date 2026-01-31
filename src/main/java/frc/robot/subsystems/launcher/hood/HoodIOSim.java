package frc.robot.subsystems.launcher.hood;

import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class HoodIOSim extends HoodIOTalonFX {
    private TalonFXSimState simState;
    private DCMotorSim sim = new DCMotorSim(
        LinearSystemId.createDCMotorSystem(
            DCMotor.getKrakenX60(1),
            0.01,
            1.0
        ),
        DCMotor.getKrakenX60(1)
    );

    public HoodIOSim() {
        this.simState = this.motor.getSimState();
    }

    @Override
    public void updateInputs(HoodIOInputs inputs) {
        simState.setSupplyVoltage(RobotController.getBatteryVoltage());

        sim.setInputVoltage(simState.getMotorVoltage());
        sim.update(0.02);

        simState.setRotorVelocity(sim.getAngularVelocity());
        
        super.updateInputs(inputs);
    }
}
