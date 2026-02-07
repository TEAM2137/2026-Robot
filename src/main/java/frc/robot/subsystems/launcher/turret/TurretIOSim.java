package frc.robot.subsystems.launcher.turret;

import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class TurretIOSim extends TurretIOTalonFX {
    private TalonFXSimState simState;
    
    private DCMotorSim sim = new DCMotorSim(
        LinearSystemId.createDCMotorSystem(
            DCMotor.getKrakenX60(1),
            0.01,
            Constants.gearing
        ),
        DCMotor.getKrakenX60(1)
    );

    public TurretIOSim() {
        this.simState = this.motor.getSimState();
        this.simState.setMotorType(TalonFXSimState.MotorType.KrakenX60);
    }

    @Override
    public void updateInputs(TurretIOInputs inputs) {
        simState.setSupplyVoltage(RobotController.getBatteryVoltage());
        
        sim.setInputVoltage(simState.getMotorVoltage());
        sim.update(0.02);

        simState.setRawRotorPosition(sim.getAngularPosition().times(Constants.gearing));
        simState.setRotorVelocity(sim.getAngularVelocity().times(Constants.gearing));

        super.updateInputs(inputs);
    }
}
