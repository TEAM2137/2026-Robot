package frc.robot;

import java.util.function.Supplier;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.math.geometry.Pose2d;

public class AutoRoutines {
    /** scores 1,000,000 fuel in the opposing alliance's hub, then climbs to L3 */
    public static UnregisteredAuto oneMillionFuelAuto(AutoRoutine auto, AutoTrajectory[] trajectories, RobotContainer robot) {
        auto.active().onTrue(trajectories[0].resetOdometry().andThen(trajectories[0].cmd()));
        return new UnregisteredAuto(auto, () -> trajectories[0].getInitialPose().orElse(null));
    }

    /** register all the autos defined above */
    public static void registerAutos(AutoFactory factory, AutoRegistry autos) {
        autos.add("One Million Fuel Auto", "millionFuel", 1, AutoRoutines::oneMillionFuelAuto);
    }
    
    @FunctionalInterface
    public interface AutoRegistry {
        /**
         * @param name The name of the path as seen in the chooser on the driver station dashboard
         * @param id The name of the auto routine in choreo (no spaces allowed)
         * @param startPose The robot's starting pose for the auto
         * @param builder A function that creates the auto given an AutoRoutine and RobotContainer instance
         */
        public void add(String name, String id, int splits, AutoBuilder builder);
    }
    
    @FunctionalInterface
    public interface AutoBuilder {
        /**
         * @param routine The auto routine instance
         * @param trajectories List of trajectories determined by the number of splits defined during registration
         * @param robot The robot container isntance
         * @return An UnregisteredAuto containing the modified routine and start pose
         */
        public UnregisteredAuto build(AutoRoutine routine, AutoTrajectory[] trajectories, RobotContainer robot);
    }

    /** I just needed something to store an auto and a start pose simultaneously */
    public record UnregisteredAuto(AutoRoutine routine, Supplier<Pose2d> startPose) {}
}
