package frc.robot.subsystems.indexer;

import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class IndexerIOSim extends IndexerIOTalonFX {
    private TalonFXSimState indexerSimState;
    private DCMotorSim indexerSim = new DCMotorSim(
        LinearSystemId.createDCMotorSystem(
            DCMotor.getKrakenX60(1),
            0.01,
            Constants.indexerGearing
        ),
        DCMotor.getKrakenX60(1)
    );

    private TalonFXSimState feederSimState;
    private DCMotorSim feederSim = new DCMotorSim(
        LinearSystemId.createDCMotorSystem(
            DCMotor.getKrakenX60(1),
            0.01,
            Constants.indexerGearing
        ),
        DCMotor.getKrakenX60(1)
    );

    public IndexerIOSim() {
        this.indexerSimState = this.indexer.getSimState();
        this.feederSimState = this.feeder.getSimState();
    }

    @Override
    public void updateInputs(IndexerIOInputs inputs) {
        indexerSimState.setSupplyVoltage(RobotController.getBatteryVoltage());
        indexerSim.setInputVoltage(indexerSimState.getMotorVoltage());
        indexerSim.update(0.02);

        feederSimState.setSupplyVoltage(RobotController.getBatteryVoltage());
        feederSim.setInputVoltage(indexerSimState.getMotorVoltage());
        feederSim.update(0.02);

        super.updateInputs(inputs);
    }
}
