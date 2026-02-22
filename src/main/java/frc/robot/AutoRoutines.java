package frc.robot;

import java.util.function.Supplier;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.autoalign.AutoAlignCommand;


public class AutoRoutines {
    /** starts on the top, drives into the neutral zone fuel, returns to shoot, repeats twice, and climbs */
    public static UnregisteredAuto twoCycleAuto(AutoRoutine auto, AutoTrajectory[] trajectories, RobotContainer robot) {
        // reset odometry and start first cycle
        auto.active().onTrue(trajectories[0].resetOdometry().andThen(trajectories[0].cmd()));

        // drive to scoring position after intaking
        trajectories[0].done().onTrue(trajectories[1].cmd());

        // start first scoring sequence
        trajectories[1].done().onTrue(new SequentialCommandGroup(
            launchAllFuel(robot), // launch all of the robot's fuel
            trajectories[2].cmd().asProxy() // start second cycle
        ));

        // drive to scoring position after intaking
        trajectories[2].done().onTrue(trajectories[3].cmd());

        // start second scoring sequence
        trajectories[3].done().onTrue(new SequentialCommandGroup(
            launchAllFuel(robot), // launch all of the robot's fuel
            trajectories[4].cmd().asProxy() // drive to the tower for climb
        ));

        // return the modified routine and start pose
        return new UnregisteredAuto(auto, () -> trajectories[0].getInitialPose().orElse(null));
    }

    public static UnregisteredAuto questionableAuto(AutoRoutine auto, AutoTrajectory[] trajectories, RobotContainer robot) {
        // reset odometry and start first cycle
        auto.active().onTrue(trajectories[0].resetOdometry().andThen(trajectories[0].cmd()));

        trajectories[0].done().onTrue(trajectories[1].cmd());
        //Add intake deploy and run

        trajectories[1].done().onTrue(trajectories[2].cmd());
        launchAllFuel(robot); // TODO: fix all of these command constructions

        trajectories[2].done().onTrue(trajectories[3].cmd());
        //retract and stop intake

        trajectories[3].done().onTrue(trajectories[4].cmd());
        launchAllFuel(robot);

        trajectories[4].done().onTrue(trajectories[5].cmd());
        //climb

        // return the modified routine and start pose
        return new UnregisteredAuto(auto, () -> trajectories[0].getInitialPose().orElse(null));
    }

    public static UnregisteredAuto secondCycleNear(AutoRoutine auto, AutoTrajectory[] trajectories, RobotContainer robot) {
        // reset odometry and start first cycle
        auto.active().onTrue(trajectories[0].resetOdometry().andThen(new AutoAlignCommand(/*align to position across bump */)).andThen(trajectories[1].cmd()));
        //before running 2, start intaking
        trajectories[1].done().onTrue(trajectories[2].cmd());
        launchAllFuel(robot);
        trajectories[2].done().onTrue(trajectories[3].cmd());
        trajectories[3].done().onTrue(trajectories[4].cmd());
        launchAllFuel(robot);
        trajectories[4].done().onTrue(trajectories[5].cmd());
        trajectories[5].done().onTrue(trajectories[6].cmd());
        trajectories[6].done().onTrue(trajectories[7].cmd());
        trajectories[7].done().onTrue(trajectories[8].cmd());
        trajectories[8].done().onTrue(trajectories[9].cmd());
        trajectories[9].done().onTrue(trajectories[10].cmd());
        trajectories[10].done().onTrue(trajectories[11].cmd());

        // return the modified routine and start pose
        return new UnregisteredAuto(auto, () -> trajectories[0].getInitialPose().orElse(null));
    }

    public static UnregisteredAuto depotHubClimb(AutoRoutine auto, AutoTrajectory[] trajectories, RobotContainer robot) {
        // reset odometry, start intaking, and start first path
        auto.active().onTrue(trajectories[0].resetOdometry().andThen(trajectories[1].cmd()));
        //go toward bump and launch fuel, stop intake
        trajectories[1].done().onTrue(trajectories[2].cmd());
        launchAllFuel(robot);
        //auto align over bump
        trajectories[2].done().onTrue(trajectories[3].cmd());
        //enable intake and start collecting again
        trajectories[3].done().onTrue(trajectories[4].cmd());
        //stop intake and move toward bump
        trajectories[4].done().onTrue(trajectories[5].cmd());
        //auto align over bump and launch

        // return the modified routine and start pose
        return new UnregisteredAuto(auto, () -> trajectories[0].getInitialPose().orElse(null));
    }

    public static Command launchAllFuel(RobotContainer robot) {
        return new SequentialCommandGroup(
            robot.launcher.startLaunching().asProxy(), // start launching fuel
            Commands.waitSeconds(2), // wait for all fuel to be fired
            robot.launcher.stopLaunching().asProxy() // stop launching fuel
        );
    }

    /** register all the autos defined above */
    public static void registerAutos(AutoFactory factory, AutoRegistry autos) {
        autos.add("Two Cycle", "twoCycle", 5, AutoRoutines::twoCycleAuto);
        autos.add("Questionable (Named by Avery)", "questionable", 6, AutoRoutines::questionableAuto);
        autos.add("Second Cycle Near", "secondCycleNear", 9, AutoRoutines::secondCycleNear);
        autos.add("Depot, Hub, Climb", "depotHubClimb", 6, AutoRoutines::depotHubClimb);
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
