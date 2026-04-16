package frc.robot;

import java.util.function.Supplier;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.launcher.LaunchState;
import frc.robot.subsystems.launcher.flywheel.Flywheel;

public class AutoRoutines {
    public static UnregisteredAuto depotAuto(AutoRoutine auto, AutoTrajectory[] trajectories, RobotContainer robot) {
        AutoTrajectory overBump1 = trajectories[0];
        AutoTrajectory intakePass = trajectories[1];
        AutoTrajectory returnFromIntaking = trajectories[2];
        AutoTrajectory overBump2 = trajectories[3];
        AutoTrajectory lineupForDepot = trajectories[4];
        AutoTrajectory backupIntoDepot = trajectories[5];
        AutoTrajectory driveOverDepot = trajectories[6];
        AutoTrajectory driveOut = trajectories[7];

        auto.active().onTrue(overBump1.resetOdometry().andThen(overBump1.cmd()));
        auto.active().onTrue(robot.intake.startIntakeSequence());
        overBump1.done().onTrue(intakePass.cmd());

        intakePass.done().onTrue(returnFromIntaking.cmd());
        returnFromIntaking.done().onTrue(overBump2.cmd());

        overBump2.done().onTrue(new SequentialCommandGroup(
            robot.launcher.setState(LaunchState.LAUNCH),
            Commands.waitSeconds(Flywheel.Constants.SPIN_UP_TIME),
            robot.intake.runRollers(),
            new SequentialCommandGroup(
                robot.indexer.run().repeatedly().onlyWhile(robot.launcher.getTurret().isAtTarget()),
                robot.indexer.stop()
            ).repeatedly()
        ));
        overBump2.done().onTrue(lineupForDepot.cmd());
        overBump2.doneDelayed(Flywheel.Constants.SPIN_UP_TIME + 0.5).onTrue(robot.intake.agitate());

        lineupForDepot.done().onTrue(robot.intake.startIntakeSequence());
        lineupForDepot.done().onTrue(backupIntoDepot.cmd());

        backupIntoDepot.done().onTrue(driveOverDepot.cmd());

        driveOverDepot.done().onTrue(new SequentialCommandGroup(
            Commands.waitSeconds(0.3),
            robot.intake.agitate()
        ));
        driveOverDepot.done().onTrue(driveOut.cmd());

        return new UnregisteredAuto(auto, () -> trajectories[0].getInitialPose().orElse(null));
    }

    public static UnregisteredAuto cycleRight(AutoRoutine auto, AutoTrajectory[] trajectories, RobotContainer robot) {
        AutoTrajectory overBump1 = trajectories[0];
        AutoTrajectory intakePass = trajectories[1];
        AutoTrajectory returnFromIntaking = trajectories[2];
        AutoTrajectory overBump2 = trajectories[3];
        AutoTrajectory overBump3 = trajectories[4];
        AutoTrajectory secondIntakePass = trajectories[5];

        auto.active().onTrue(overBump1.resetOdometry().andThen(overBump1.cmd()));
        auto.active().onTrue(new SequentialCommandGroup(
            robot.intake.deploy(),
            robot.intake.runRollers()
        ));
        overBump1.done().onTrue(intakePass.cmd());

        intakePass.done().onTrue(returnFromIntaking.cmd());
        returnFromIntaking.done().onTrue(overBump2.cmd());

        overBump2.atTimeBeforeEnd(0.8).onTrue(new SequentialCommandGroup(
            robot.launcher.setState(LaunchState.LAUNCH),
            Commands.waitSeconds(Flywheel.Constants.SPIN_UP_TIME),
            new ParallelCommandGroup(
                new SequentialCommandGroup(
                    robot.indexer.run().repeatedly().onlyWhile(robot.launcher.getTurret().isAtTarget()),
                    robot.indexer.stop()
                ).repeatedly(),
                robot.intake.agitate()
            ).withTimeout(7.0),
            robot.launcher.setState(LaunchState.AUTOMATIC),
            robot.intake.deploy(),
            robot.intake.runRollers(),
            overBump3.cmd().asProxy()
        ));

        overBump3.done().onTrue(secondIntakePass.cmd());

        return new UnregisteredAuto(auto, () -> trajectories[0].getInitialPose().orElse(null));
    }

    public static UnregisteredAuto rightDepot(AutoRoutine auto, AutoTrajectory[] trajectories, RobotContainer robot) {
        AutoTrajectory overBump1 = trajectories[0];
        AutoTrajectory intakePass = trajectories[1];
        AutoTrajectory returnFromIntaking = trajectories[2];
        AutoTrajectory overBump2 = trajectories[3];
        AutoTrajectory lineupForDepot = trajectories[4];
        AutoTrajectory backupIntoDepot = trajectories[5];
        AutoTrajectory driveOverDepot = trajectories[6];
        AutoTrajectory driveOut = trajectories[7];

        auto.active().onTrue(overBump1.resetOdometry().andThen(overBump1.cmd()));
        auto.active().onTrue(new SequentialCommandGroup(
            robot.intake.deploy(),
            robot.intake.runRollers()
        ));
        overBump1.done().onTrue(intakePass.cmd());

        intakePass.done().onTrue(returnFromIntaking.cmd());
        returnFromIntaking.done().onTrue(overBump2.cmd());

        overBump2.done().onTrue(new SequentialCommandGroup(
            robot.launcher.setState(LaunchState.LAUNCH),
            Commands.waitSeconds(Flywheel.Constants.SPIN_UP_TIME),
            robot.intake.runRollers(),
            new SequentialCommandGroup(
                robot.indexer.run().repeatedly().onlyWhile(robot.launcher.getTurret().isAtTarget()),
                robot.indexer.stop()
            ).repeatedly()
        ));
        overBump2.done().onTrue(lineupForDepot.cmd());

        lineupForDepot.done().onTrue(backupIntoDepot.cmd());
        backupIntoDepot.done().onTrue(driveOverDepot.cmd());

        driveOverDepot.done().onTrue(new SequentialCommandGroup(
            Commands.waitSeconds(0.3),
            robot.intake.agitate()
        ));
        driveOverDepot.done().onTrue(driveOut.cmd());

        return new UnregisteredAuto(auto, () -> trajectories[0].getInitialPose().orElse(null));
    }

    /** register all the autos defined above */
    public static void registerAutos(AutoFactory factory, AutoRegistry autos) {
        // comp autos
        autos.add("Depot", "depot", 8, false, AutoRoutines::depotAuto);
        autos.add("Right Depot", "rightDepot", 8, false, AutoRoutines::rightDepot);
        autos.add("Cycle Right", "cycleRight", 6, false, AutoRoutines::cycleRight);
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
