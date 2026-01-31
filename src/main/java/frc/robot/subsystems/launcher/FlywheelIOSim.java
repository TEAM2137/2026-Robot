package frc.robot.subsystems.launcher;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

public class FlywheelIOSim extends FlywheelIOTalonFX {
    private TalonFXSimState leaderSimState;
    private TalonFXSimState followerSimState;
    
    private FlywheelSim sim = new FlywheelSim(
        LinearSystemId.createFlywheelSystem(
            DCMotor.getKrakenX60(2),
            0.01,
            Constants.gearing
        ),
        DCMotor.getKrakenX60(2)
    );

    public FlywheelIOSim() {
        this.leaderSimState = this.leader.getSimState();
        this.followerSimState = this.follower.getSimState();
    }

    @Override
    public void updateInputs(FlywheelIOInputs inputs) {
        leaderSimState.setSupplyVoltage(RobotController.getBatteryVoltage());
        followerSimState.setSupplyVoltage(RobotController.getBatteryVoltage());
        
        sim.setInputVoltage(leaderSimState.getMotorVoltage());
        sim.update(0.02);

        leaderSimState.setRotorVelocity(sim.getAngularVelocity().in(RotationsPerSecond));

        super.updateInputs(inputs);
    }
}
