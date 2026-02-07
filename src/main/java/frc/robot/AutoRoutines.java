package frc.robot;

import java.util.function.Supplier;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class AutoRoutines {
    /** starts on the top, drives into the neutral zone fuel, returns to shoot, repeats twice, and climbs */
    public static UnregisteredAuto twoCycleAuto(AutoRoutine auto, AutoTrajectory[] trajectories, RobotContainer robot) {
        // reset odometry and start first cycle
        auto.active().onTrue(trajectories[0].resetOdometry().andThen(trajectories[0].cmd()));

        // drive to scoring position after intaking
        trajectories[0].done().onTrue(trajectories[1].cmd());

        // start first scoring sequence
        trajectories[1].done().onTrue(new SequentialCommandGroup(
            robot.launcher.startLaunching(), // start launching fuel
            Commands.waitSeconds(2), // wait for all fuel to be fired
            robot.launcher.stopLaunching(), // stop launching fuel
            trajectories[2].cmd().asProxy() // start second cycle
        ));

        // drive to scoring position after intaking
        trajectories[2].done().onTrue(trajectories[3].cmd());

        // start second scoring sequence
        trajectories[3].done().onTrue(new SequentialCommandGroup(
            robot.launcher.startLaunching(), // start launching fuel
            Commands.waitSeconds(2), // wait for all fuel to be fired
            robot.launcher.stopLaunching(), // stop launching fuel
            trajectories[4].cmd().asProxy() // drive to the tower for climb
        ));

        // return the modified routine and start pose
        return new UnregisteredAuto(auto, () -> trajectories[0].getInitialPose().orElse(null));
    }

    /** register all the autos defined above */
    public static void registerAutos(AutoFactory factory, AutoRegistry autos) {
        autos.add("Two Cycle", "twoCycle", 5, AutoRoutines::twoCycleAuto);
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
