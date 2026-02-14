package frc.robot;

import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;
import java.io.Writer;
import java.lang.reflect.Type;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Optional;
import java.util.Set;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.google.gson.Gson;
import com.google.gson.GsonBuilder;
import com.google.gson.reflect.TypeToken;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation.MatchType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.AutoRoutines.UnregisteredAuto;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.Alerts;

public class Autonomous {
    private record RegisteredAuto(String name, Supplier<Pose2d> startPose, AutoRoutine auto) {}

    private final HashMap<String, RegisteredAuto> autos = new HashMap<>();
    private final LoggedDashboardChooser<RegisteredAuto> autoChooser = new LoggedDashboardChooser<>("Auto Chooser");
    private final LoggedDashboardChooser<Command> sysIdChooser = new LoggedDashboardChooser<>("SysID Command Chooser");

    private final AutoFactory factory;

    public Autonomous(RobotContainer robot) {
        Drive drive = robot.drive;

        // Create auto factory
        this.factory = new AutoFactory(
            drive::getPose, // A function that returns the current robot pose
            drive::setPose, // A function that resets the current robot pose to the provided Pose2d
            drive::followTrajectory, // The drive subsystem trajectory follower
            true, // If alliance flipping should be enabled
            drive // The drive subsystem
        );

        // Register autos from AutoRoutines
        AutoRoutines.registerAutos(this.factory, (name, id, splits, builder) -> {
            AutoRoutine routine = this.factory.newRoutine(name);
            UnregisteredAuto unregistered = builder.build(routine, this.loadTrajectories(routine, id, splits), robot);
            RegisteredAuto auto = new RegisteredAuto(name, unregistered.startPose(), unregistered.routine());
            this.autos.put(name, auto);
        });
        
        // Create the auto chooser
        this.autoChooser.addDefaultOption("None", null);
        this.autos.forEach((name, auto) -> autoChooser.addOption(name, auto));

        // Create the sysId command chooser
        this.sysIdChooser.addDefaultOption("None", null);
        this.sysIdChooser.addOption("Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
        this.sysIdChooser.addOption("Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
        this.sysIdChooser.addOption("Drive SysId (Quasistatic Forward)", drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
        this.sysIdChooser.addOption("Drive SysId (Quasistatic Reverse)", drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
        this.sysIdChooser.addOption("Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
        this.sysIdChooser.addOption("Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

        // Assign auto commands to autonomous trigger
        RobotModeTriggers.autonomous().whileTrue(this.getAutoCommand());
        RobotModeTriggers.autonomous().onTrue(Commands.runOnce(() -> this.saveSetupScore(robot)));
        Alerts.add("No Auto Selected", AlertType.kWarning, this::isNoAutoSelected);
    }

    public AutoTrajectory[] loadTrajectories(AutoRoutine routine, String id, int numSplits) {
        AutoTrajectory[] trajectories = new AutoTrajectory[numSplits];
        for (int i = 0; i < numSplits; i++) { trajectories[i] = routine.trajectory(id, i); }
        return trajectories;
    }

    public Command getAutoCommand() {
        return Commands.defer(() -> {
            if (autoChooser.get() != null) return autoChooser.get().auto().cmd().asProxy();
            if (sysIdChooser.get() != null) return sysIdChooser.get().asProxy();
            return Commands.none();
        }, Set.of());
    }

    public boolean isNoAutoSelected() {
        return autoChooser.get() == null && sysIdChooser.get() == null;
    }

    public Optional<Pose2d> getStartPose() {
        RegisteredAuto auto = autoChooser.get();
        if (auto == null) return Optional.empty();
        return Optional.ofNullable(auto.startPose.get());
    }

    public record SetupScore(String letterGrade, double score, double gpa, int matchNumber, MatchType matchType, String eventID) {}

    public void saveSetupScore(RobotContainer robot) {
        // only log the score if the selected auto has a valid start pose
        Pose2d startPose = getStartPose().orElse(null);
        if (startPose == null) return;
        
        // only log the score if we are in a real match
        boolean isOfficial = DriverStation.isFMSAttached()
                && DriverStation.getMatchType() != MatchType.Practice
                && DriverStation.getMatchType() != MatchType.None;
        if (!isOfficial) return;

        // grab setup score, grade, and gpa
        double score = getRawSetupScore(robot.drive.getPose(), startPose);
        int scoreRounded = (int) Math.round(score);
        String letterGrade = getLetterGradeFor(scoreRounded);
        double gpa = getGPAFor(scoreRounded);

        // grab FMS data
        int matchNumber = DriverStation.getMatchNumber();
        MatchType matchType = DriverStation.getMatchType();
        String eventID = DriverStation.getEventName();

        // create gson instance
        String filePath = "scores.json";
        Gson gson = new GsonBuilder().setPrettyPrinting().create();

        // create score list and add current setup score
        ArrayList<SetupScore> setupScores = new ArrayList<>();
        setupScores.add(new SetupScore(letterGrade, score, gpa, matchNumber, matchType, eventID));

        // read previous scores and add them to the list
        try (FileReader reader = new FileReader(filePath)) {
            Type listType = new TypeToken<List<SetupScore>>(){}.getType();
            List<SetupScore> previousScores = gson.fromJson(reader, listType);
            setupScores.addAll(previousScores);
        }
        catch (IOException e) { e.printStackTrace(); }

        // write the accumulated list to the file
        try (Writer writer = new FileWriter(filePath)) {
            gson.toJson(setupScores, writer);
            System.out.println("Successfully wrote scores list to " + filePath);
        }
        catch (IOException e) { e.printStackTrace(); }

        // calculate average GPA based on previous scores
        double average = setupScores.stream()
                .filter(ss -> ss.matchType == MatchType.Elimination)
                .mapToDouble(SetupScore::gpa)
                .average().orElse(0);
        Logger.recordOutput("Autonomous/Setup/AverageGPA", average);
    }

    private static double getRawSetupScore(Pose2d pose, Pose2d targetPose) {
        double positionError = pose.getTranslation().getDistance(targetPose.getTranslation());
        double rotationError = Math.abs(pose.getRotation().minus(targetPose.getRotation()).getDegrees());

        Logger.recordOutput("Autonomous/Setup/Target", targetPose);
        Logger.recordOutput("Autonomous/Setup/PosError", positionError);
        Logger.recordOutput("Autonomous/Setup/RotError", rotationError);

        return 100 * Math.exp(-(positionError + 0.008 * rotationError));
    }

    public static String getSetupScore(Pose2d pose, Pose2d targetPose) {
        double rawScore = getRawSetupScore(pose, targetPose);
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

    private static double getGPAFor(int scoreRounded) {
        return switch (getLetterGradeFor(scoreRounded)) {
            case "A+" -> 4.0;
            case "A" -> 4.0;
            case "A-" -> 3.7;
            case "B+" -> 3.3;
            case "B" -> 3.0;
            case "B-" -> 2.7;
            case "C+" -> 2.3;
            case "C" -> 2.0;
            case "C-" -> 1.7;
            case "D+" -> 1.3;
            case "D" -> 1.0;
            case "D-" -> 0.7;
            default -> 0.0;
        };
    }
}
