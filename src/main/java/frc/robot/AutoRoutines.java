package frc.robot;

import java.util.function.Supplier;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.autoalign.AutoAlignCommand;
import frc.robot.subsystems.intake.Intake;


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

        trajectories[0].done().onTrue(robot.intake.startIntakeSequence().andThen(trajectories[1].cmd()));
        

        trajectories[1].done().onTrue(trajectories[2].cmd());
        launchAllFuel(robot); // TODO: fix all of these command constructions

        trajectories[2].done().onTrue(robot.intake.setRollerVoltage(0).andThen(robot.intake.retract()).andThen(trajectories[3].cmd()));
        //retract and stop intake

        trajectories[3].done().onTrue(trajectories[4].cmd().andThen(launchAllFuel(robot)));
        //launch fuel

        trajectories[4].done().onTrue(trajectories[5].cmd().andThen(startAutoClimbSequence(robot)));
        //climb

        // return the modified routine and start pose
        return new UnregisteredAuto(auto, () -> trajectories[0].getInitialPose().orElse(null));
    }

    public static UnregisteredAuto secondCycleNear(AutoRoutine auto, AutoTrajectory[] trajectories, RobotContainer robot) {
        // reset odometry and start first cycle
        auto.active().onTrue(trajectories[0].resetOdometry()
        .andThen(new AutoAlignCommand()
        .withTargetPose(new Pose2d(5.81532, 5.90507, new Rotation2d(-Math.PI / 4)))
        .withSpeedLimit(5))
        .andThen(trajectories[1].cmd())
        .andThen(robot.intake.startIntakeSequence()));
        trajectories[1].done().onTrue(trajectories[2].cmd());
        trajectories[2].done().onTrue(trajectories[3].cmd().andThen(robot.intake.setRollerVoltage(0)));
        trajectories[3].done().onTrue(trajectories[4].cmd().andThen(new AutoAlignCommand()

        ));

        //auto align over bump, stop, launch (replaces split 5)

        //auto align back to NZ, start intake (replaces split 6)

        trajectories[6].done().onTrue(trajectories[7].cmd());
        //stop intake

        //auto align over bump, launch (replaces split 8)


        // return the modified routine and start pose
        return new UnregisteredAuto(auto, () -> trajectories[0].getInitialPose().orElse(null));
    }

    public static UnregisteredAuto depotHubClimb(AutoRoutine auto, AutoTrajectory[] trajectories, RobotContainer robot) {
        // reset odometry, start intaking, and start path 1
        auto.active().onTrue(robot.intake.startIntakeSequence().andThen(trajectories[0].resetOdometry()).andThen(trajectories[1].cmd()));
        //go toward bump and launch fuel, stop intake
        trajectories[1].done().onTrue(new ParallelCommandGroup(
            robot.intake.setRollerVoltage(0),
            new SequentialCommandGroup(
            launchAllFuel(robot),
            Commands.waitSeconds(5),

            new AutoAlignCommand()
            .withTargetPose(new Pose2d(5.71598, 5.60704, new Rotation2d(-Math.PI / 4)))
            .withAccelerationLimit(3)
            .withSpeedLimit(5),
            robot.intake.setRollerVoltage(Intake.Constants.rollerVoltage),
            trajectories[3].cmd()
            )));

        //Auto align over bump, launch, go to tower, climb
        trajectories[3].done().onTrue
        (new SequentialCommandGroup(
            robot.intake.setRollerVoltage(0),
            new AutoAlignCommand()
            .withTargetPose(new Pose2d(3.16147, 2.45647, new Rotation2d(-Math.PI * 3 / 4)))
            .withAccelerationLimit(3)
            .withSpeedLimit(5),
            launchAllFuel(robot),
            trajectories[5].cmd(),
            startAutoClimbSequence(robot)));

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
    public static Command startAutoClimbSequence(RobotContainer robot) {
        return new SequentialCommandGroup(
            robot.climber.extendClimb(),
            robot.climber.deployUpperHooks(),
            robot.climber.retractClimb()
        );
    }

    public static UnregisteredAuto outpostAuto(AutoRoutine auto, AutoTrajectory[] trajectories, RobotContainer robot) {
        auto.active().onTrue(trajectories[0].resetOdometry().andThen(trajectories[0].cmd()));
        trajectories[0].done().onTrue(trajectories[1].cmd());

        trajectories[1].atTimeBeforeEnd(0.4).onTrue(new SequentialCommandGroup(
            robot.intake.deploy(),
            robot.intake.runRollers()
        ));
        trajectories[1].done().onTrue(trajectories[2].cmd());

        trajectories[2].done().onTrue(trajectories[3].cmd());
        trajectories[3].done().onTrue(trajectories[4].cmd());
        trajectories[4].done().onTrue(trajectories[5].cmd());

        trajectories[5].atTimeBeforeEnd(1).onTrue(new SequentialCommandGroup(
            robot.launcher.startLaunching(),
            Commands.waitSeconds(0.2),
            robot.intake.runRollers(),
            new SequentialCommandGroup(
                robot.indexer.run().repeatedly().onlyWhile(robot.launcher.getTurret().isAtTarget()),
                robot.indexer.stop()
            ).repeatedly()
        ));

        trajectories[5].done().onTrue(new SequentialCommandGroup(
            Commands.waitSeconds(1),
            robot.intake.agitate()
        ));

        return new UnregisteredAuto(auto, () -> trajectories[0].getInitialPose().orElse(null));
    }

    public static UnregisteredAuto depotTest(AutoRoutine auto, AutoTrajectory[] trajectories, RobotContainer robot) {
        auto.active().onTrue(trajectories[0].resetOdometry().andThen(trajectories[0].cmd()));
        trajectories[0].done().onTrue(trajectories[1].cmd());

        trajectories[1].atTimeBeforeEnd(0.4).onTrue(new SequentialCommandGroup(
            robot.intake.deploy(),
            robot.intake.runRollers()
        ));
        trajectories[1].done().onTrue(trajectories[2].cmd());

        trajectories[2].done().onTrue(trajectories[3].cmd());
        trajectories[3].done().onTrue(trajectories[4].cmd());
        trajectories[4].done().onTrue(trajectories[5].cmd());

        trajectories[5].atTimeBeforeEnd(0.2).onTrue(new SequentialCommandGroup(
            robot.launcher.startLaunching(),
            Commands.waitSeconds(0.2),
            robot.intake.runRollers(),
            new SequentialCommandGroup(
                robot.indexer.run().repeatedly().onlyWhile(robot.launcher.getTurret().isAtTarget()),
                robot.indexer.stop()
            ).repeatedly()
        ));

        trajectories[5].done().onTrue(trajectories[6].cmd());

        trajectories[6].done().onTrue(new SequentialCommandGroup(
            Commands.waitSeconds(1),
            robot.intake.agitate()
        ));

        return new UnregisteredAuto(auto, () -> trajectories[0].getInitialPose().orElse(null));
    }

    /** register all the autos defined above */
    public static void registerAutos(AutoFactory factory, AutoRegistry autos) {
        autos.add("Outpost", "auto", 6, false, AutoRoutines::outpostAuto);
        autos.add("Depot", "depotTest", 7, false, AutoRoutines::depotTest);
        // autos.add("Two Cycle", "twoCycle", 5, false, AutoRoutines::twoCycleAuto);
        // autos.add("Questionable", "questionable", 6, AutoRoutines::questionableAuto);
        // autos.add("Second Cycle Near", "secondCycleNear", 9, AutoRoutines::secondCycleNear);
        // autos.add("Depot, Hub, Climb", "depotHubClimb", 6, AutoRoutines::depotHubClimb);
    }

    @FunctionalInterface
    public interface AutoRegistry {
        /**
         * @param name The name of the path as seen in the chooser on the driver station dashboard
         * @param id The name of the auto routine in choreo (no spaces allowed)
         * @param startPose The robot's starting pose for the auto
         * @param builder A function that creates the auto given an AutoRoutine and RobotContainer instance
         */
        public void add(String name, String id, int splits, boolean doesClimb, AutoBuilder builder);
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
