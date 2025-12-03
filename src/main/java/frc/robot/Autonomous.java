package frc.robot;

import java.util.HashMap;
import java.util.Map;
import java.util.Optional;
import java.util.Set;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.drive.Drive;

public class Autonomous {
    private final Map<String, Pose2d> startPoses = new HashMap<>();

    private final LoggedDashboardChooser<Command> sysIdCommandChooser;

    private final RobotContainer robot;
    private final Drive drive;

    public Autonomous(RobotContainer robot) {
        this.robot = robot;
        this.drive = robot.drive;

        // Create the sysId command chooser
        this.sysIdCommandChooser = new LoggedDashboardChooser<>("SysID Command Chooser");
        this.sysIdCommandChooser.addDefaultOption("None", null);

        // Create the auto chooser
        this.registerAutos();

        // Assign auto commands to autonomous trigger
        RobotModeTriggers.autonomous().whileTrue(getSelectedAuto());
    }

    /** @return A command to schedule the auto selected on the chooser */
    public Command getSelectedAuto() {
        return Commands.defer(() -> {
            if (sysIdCommandChooser.get() != null) return sysIdCommandChooser.get().asProxy();
            else return Commands.none();
        }, Set.of());
    }

    public Optional<Pose2d> getStartPose() {
        return Optional.empty();
    }

    /** Adds autos to the chooser */
    public void registerAutos() {
        // SysId routines
        sysIdCommandChooser.addOption("Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
        sysIdCommandChooser.addOption("Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
        sysIdCommandChooser.addOption("Drive SysId (Quasistatic Forward)", drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
        sysIdCommandChooser.addOption("Drive SysId (Quasistatic Reverse)", drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
        sysIdCommandChooser.addOption("Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
        sysIdCommandChooser.addOption("Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));
    }

    public static String getSetupScore(Pose2d pose, Pose2d targetPose) {
        double positionError = pose.getTranslation().getDistance(targetPose.getTranslation());
        double rotationError = Math.abs(pose.getRotation().minus(targetPose.getRotation()).getDegrees());

        Logger.recordOutput("Autonomous/Setup/PosError", positionError);
        Logger.recordOutput("Autonomous/Setup/RotError", rotationError);

        double rawScore = 100 * Math.exp(-(positionError + 0.008 * rotationError));
        int scoreRounded = (int) Math.round(rawScore);
        double scoreNearestHundredth = ((int) (rawScore * 10.0)) / 10.0;

        String letterGrade = getLetterGradeFor(scoreRounded);
        return letterGrade + ": " + scoreNearestHundredth + "%";
    }

    private static String getLetterGradeFor(int scoreRounded) {
        String letterGrade = "F";
        if (scoreRounded >= 97) letterGrade = "A+";
        else if (scoreRounded >= 93) letterGrade = "A";
        else if (scoreRounded >= 90) letterGrade = "A-";
        else if (scoreRounded >= 87) letterGrade = "B+";
        else if (scoreRounded >= 83) letterGrade = "B";
        else if (scoreRounded >= 80) letterGrade = "B-";
        else if (scoreRounded >= 77) letterGrade = "C+";
        else if (scoreRounded >= 73) letterGrade = "C";
        else if (scoreRounded >= 70) letterGrade = "C-";
        else if (scoreRounded >= 67) letterGrade = "D+";
        else if (scoreRounded >= 63) letterGrade = "D";
        else if (scoreRounded >= 60) letterGrade = "D-";
        return letterGrade;
    }
}
