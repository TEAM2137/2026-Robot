package frc.robot;

import java.util.function.Supplier;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.indexer.Indexer;
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

    public static UnregisteredAuto outpostAuto(AutoRoutine auto, AutoTrajectory[] trajectories, RobotContainer robot) {
        auto.active().onTrue(trajectories[0].resetOdometry().andThen(trajectories[0].cmd()));
        trajectories[0].done().onTrue(trajectories[1].cmd());

        trajectories[1].atTimeBeforeEnd(0.7).onTrue(new SequentialCommandGroup(
            robot.intake.deploy(),
            robot.intake.runRollers()
        ));
        trajectories[1].done().onTrue(trajectories[2].cmd());

        trajectories[2].done().onTrue(trajectories[3].cmd());
        trajectories[3].done().onTrue(trajectories[4].cmd());
        trajectories[4].done().onTrue(trajectories[5].cmd());

        trajectories[5].atTimeBeforeEnd(0.6).onTrue(new SequentialCommandGroup(
            robot.launcher.setState(LaunchState.LAUNCH),
            Commands.waitSeconds(0.2),
            robot.intake.runRollers(),
            new SequentialCommandGroup(
                robot.indexer.run().repeatedly().onlyWhile(robot.launcher.getTurret().isAtTarget()),
                robot.indexer.stop()
            ).repeatedly()
        ));

        trajectories[5].done().onTrue(new SequentialCommandGroup(
            Commands.waitSeconds(4.6),
            robot.intake.agitate(0.8)
        ));

        return new UnregisteredAuto(auto, () -> trajectories[0].getInitialPose().orElse(null));
    }

    public static UnregisteredAuto rightDepot(AutoRoutine auto, AutoTrajectory[] trajectories, RobotContainer robot) {
       //This is effectively just the depot auto except the center pass and starting position are on the right
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

public static UnregisteredAuto delayedDepot(AutoRoutine auto, AutoTrajectory[] trajectories, RobotContainer robot) {
       //This is effectively just the depot auto except the center pass and starting position are on the right
        AutoTrajectory overBump1 = trajectories[0];
        AutoTrajectory intakePass = trajectories[1];
        AutoTrajectory returnFromIntaking = trajectories[2];
        AutoTrajectory overBump2 = trajectories[3];
        AutoTrajectory lineupForDepot = trajectories[4];
        AutoTrajectory backupIntoDepot = trajectories[5];
        AutoTrajectory driveOverDepot = trajectories[6];
        AutoTrajectory driveOut = trajectories[7];
        
        auto.active().onTrue(new SequentialCommandGroup(
            overBump1.resetOdometry(),
            robot.launcher.setState(LaunchState.LAUNCH),
            robot.intake.runRollers(),
            robot.indexer.run(),
            Commands.waitSeconds(2),
            robot.indexer.stop(),
            robot.launcher.setState(LaunchState.DONT_LAUNCH),
            robot.intake.deploy(),
            robot.intake.runRollers(),
            overBump1.cmd()
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
    public static UnregisteredAuto farDepot(AutoRoutine auto, AutoTrajectory[] trajectories, RobotContainer robot) {
       //This is effectively just the depot auto except the center pass goes much further out
        AutoTrajectory overBump1 = trajectories[0];
        AutoTrajectory intakePass = trajectories[1];
        AutoTrajectory returnFromIntaking = trajectories[2];
        AutoTrajectory overBump2 = trajectories[3];
        AutoTrajectory lineupForDepot = trajectories[4];
        AutoTrajectory backupIntoDepot = trajectories[5];
        AutoTrajectory driveOverDepot = trajectories[6];
        AutoTrajectory driveOut = trajectories[7];
        
        auto.active().onTrue(new SequentialCommandGroup(
            overBump1.resetOdometry(),
            robot.launcher.setState(LaunchState.LAUNCH),
            robot.intake.runRollers(),
            robot.indexer.run(),
            Commands.waitSeconds(2),
            robot.indexer.stop(),
            robot.launcher.setState(LaunchState.DONT_LAUNCH),
            robot.intake.deploy(),
            robot.intake.runRollers(),
            overBump1.cmd()
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

    public static UnregisteredAuto slowClose(AutoRoutine auto, AutoTrajectory[] trajectories, RobotContainer robot) {
       //This is effectively just the depot auto except it goes slower in the middle and stays closer to the hub
        AutoTrajectory overBump1 = trajectories[0];
        AutoTrajectory intakePass = trajectories[1];
        AutoTrajectory returnFromIntaking = trajectories[2];
        AutoTrajectory overBump2 = trajectories[3];
        AutoTrajectory lineupForDepot = trajectories[4];
        AutoTrajectory backupIntoDepot = trajectories[5];
        AutoTrajectory driveOverDepot = trajectories[6];
        AutoTrajectory driveOut = trajectories[7];
        
        auto.active().onTrue(new SequentialCommandGroup(
            overBump1.resetOdometry(),
            robot.launcher.setState(LaunchState.LAUNCH),
            robot.intake.runRollers(),
            robot.indexer.run(),
            Commands.waitSeconds(2),
            robot.indexer.stop(),
            robot.launcher.setState(LaunchState.DONT_LAUNCH),
            robot.intake.deploy(),
            robot.intake.runRollers(),
            overBump1.cmd()
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
        autos.add("Outpost", "outpost", 6, false, AutoRoutines::outpostAuto);
        autos.add("Depot", "depot", 8, false, AutoRoutines::depotAuto);
        autos.add("Cycle Right", "cycleRight", 6, false, AutoRoutines::cycleRight);
        
        //Requested autos to test
        autos.add("TEST: Right Depot", "rightDepot", 8, false, AutoRoutines::rightDepot);
        autos.add("TEST: Delayed Depot", "delayedDepot", 8, false, AutoRoutines::delayedDepot);
        autos.add("TEST: Far Depot", "farDepot", 8, false, AutoRoutines::farDepot);
        autos.add("TEST: Slow Close", "slowClose", 8, false, AutoRoutines::slowClose);
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
