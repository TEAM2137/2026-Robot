package frc.robot.subsystems.launcher;

import java.util.ArrayList;
import java.util.Map;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import frc.robot.RobotContainer;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.FieldConstants;

@FunctionalInterface
public interface ShotCalculator {
    static final int SOTF_MAX_ITERATIONS = 5;
    static final double SOTF_TOF_ERROR_TOLERANCE = 0.01;
    
    static final InterpolatingDoubleTreeMap FLYWHEEL_RPM_HUB = InterpolatingDoubleTreeMap.ofEntries(
        Map.entry(1.3165, 1765.0),
        Map.entry(1.5658, 1810.0),
        Map.entry(2.0083, 1865.0),
        Map.entry(2.6245, 1921.0),
        Map.entry(3.0319, 1983.0),
        Map.entry(3.4845, 2060.0),
        Map.entry(3.8919, 2130.0),
        Map.entry(4.3134, 2183.0),
        Map.entry(5.0096, 2275.0),
        Map.entry(5.7282, 2400.0)
    );
    static final InterpolatingDoubleTreeMap HOOD_ANGLE_HUB = InterpolatingDoubleTreeMap.ofEntries(
        Map.entry(1.3165, 4.9),
        Map.entry(1.5658, 7.7),
        Map.entry(2.0083, 10.2),
        Map.entry(2.6245, 13.5),
        Map.entry(3.0319, 18.3),
        Map.entry(3.4845, 19.8),
        Map.entry(3.8919, 20.9),
        Map.entry(4.3134, 25.0),
        Map.entry(5.0096, 26.0),
        Map.entry(5.7282, 26.0)
    );

    static final InterpolatingDoubleTreeMap FLYWHEEL_RPM_PASSING = InterpolatingDoubleTreeMap.ofEntries(
        Map.entry(4.5000, 1861.0),
        Map.entry(6.9200, 2207.0),
        Map.entry(8.7000, 2450.0),
        Map.entry(12.0000, 2770.0)
    );
    static final InterpolatingDoubleTreeMap HOOD_ANGLE_PASSING = InterpolatingDoubleTreeMap.ofEntries(
        Map.entry(0.0, 22.0)
    );

    static final InterpolatingDoubleTreeMap TOF_LOOKUP = InterpolatingDoubleTreeMap.ofEntries(
        Map.entry(1.0, 0.7),
        Map.entry(1.5, 0.8),
        Map.entry(2.5, 1.0),
        Map.entry(3.0, 1.2),
        Map.entry(3.5, 1.3),
        Map.entry(4.0, 1.4),
        Map.entry(5.0, 1.5),
        Map.entry(6.0, 1.6)
    );

    static final ShotCalculator HUB = robot -> {
        Translation2d target = AllianceFlipUtil.either(FieldConstants.blueHub, FieldConstants.redHub);
        return simpleLookupShot(target, robot, FLYWHEEL_RPM_HUB, HOOD_ANGLE_HUB);
    };

    static final ShotCalculator SOTF_HUB = robot -> {
        Translation2d target = AllianceFlipUtil.either(FieldConstants.blueHub, FieldConstants.redHub);
        return simpleSOTFShot(target, robot, FLYWHEEL_RPM_HUB, HOOD_ANGLE_HUB);
    };

    static final ShotCalculator PASS_LEFT_SOTF = robot -> {
        Translation2d target = AllianceFlipUtil.either(FieldConstants.blueLeftCorner, FieldConstants.redLeftCorner);
        return simpleSOTFShot(target, robot, FLYWHEEL_RPM_PASSING, HOOD_ANGLE_PASSING);
    };
    static final ShotCalculator PASS_RIGHT_SOTF = robot -> {
        Translation2d target = AllianceFlipUtil.either(FieldConstants.blueRightCorner, FieldConstants.redRightCorner);
        return simpleSOTFShot(target, robot, FLYWHEEL_RPM_PASSING, HOOD_ANGLE_PASSING);
    };

    static final ShotCalculator PASS_LEFT = robot -> {
        Translation2d target = AllianceFlipUtil.either(FieldConstants.blueLeftCorner, FieldConstants.redLeftCorner);
        return simpleLookupShot(target, robot, FLYWHEEL_RPM_PASSING, HOOD_ANGLE_PASSING);
    };
    static final ShotCalculator PASS_RIGHT = robot -> {
        Translation2d target = AllianceFlipUtil.either(FieldConstants.blueRightCorner, FieldConstants.redRightCorner);
        return simpleLookupShot(target, robot, FLYWHEEL_RPM_PASSING, HOOD_ANGLE_PASSING);
    };

    static ShotParameters simpleLookupShot(Translation2d target, RobotContainer robot, InterpolatingDoubleTreeMap flywheelRpm, InterpolatingDoubleTreeMap hoodAngle) {
        Translation2d turretPos = robot.launcher.getTurret().getFieldSpacePose(robot).getTranslation();

        double dst = target.getDistance(turretPos);
        double dx = target.getX() - turretPos.getX();
        double dy = target.getY() - turretPos.getY();
        
        double theAngle = Math.atan2(dx, dy);

        Logger.recordOutput("ShotCalculator/TargetPos", target);
        Logger.recordOutput("ShotCalculator/TurretPos", turretPos);
        Logger.recordOutput("ShotCalculator/Distance", dst);

        return new ShotParameters(
            Rotation2d.fromRadians(theAngle).plus(Rotation2d.k180deg),
            flywheelRpm.get(dst), hoodAngle.get(dst)
        );
    }

    static ShotParameters simpleSOTFShot(Translation2d target, RobotContainer robot, InterpolatingDoubleTreeMap flywheelRpm, InterpolatingDoubleTreeMap hoodAngle) {
        Translation2d turretPos = robot.launcher.getTurret().getFieldSpacePose(robot).getTranslation();
        Translation2d turretVelocity = robot.launcher.getTurret().getFieldSpaceVelocity(robot);

        ArrayList<Pose2d> iterationPoses = new ArrayList<>(SOTF_MAX_ITERATIONS);
        ArrayList<Double> iterationDistances = new ArrayList<>(SOTF_MAX_ITERATIONS);
        ArrayList<Double> iterationTofErrors = new ArrayList<>(SOTF_MAX_ITERATIONS);

        int i = 0;
        double dst = target.getDistance(turretPos);
        double angle = 0;
        double timeOfFlight = 0;
        double tofError = 0;
        Translation2d newTarget = target;

        do {
            double previousTof = timeOfFlight;
            timeOfFlight = TOF_LOOKUP.get(dst);
            tofError = Math.abs(previousTof - timeOfFlight);
            
            newTarget = target.minus(turretVelocity.times(timeOfFlight));
            angle = Math.atan2(newTarget.getX() - turretPos.getX(), newTarget.getY() - turretPos.getY());
            dst = newTarget.getDistance(turretPos);

            iterationPoses.add(new Pose2d(newTarget, new Rotation2d()));
            iterationDistances.add(dst);
            iterationTofErrors.add(tofError);

            i++;
        }
        while (i < SOTF_MAX_ITERATIONS && tofError > SOTF_TOF_ERROR_TOLERANCE);

        Logger.recordOutput("ShotCalculator/SOTF/TargetPose", new Pose2d(newTarget, new Rotation2d()));
        Logger.recordOutput("ShotCalculator/SOTF/Distance", dst);
        Logger.recordOutput("ShotCalculator/SOTF/TimeOfFlight", timeOfFlight);
        Logger.recordOutput("ShotCalculator/SOTF/TurretVelocity", turretVelocity);
        
        Pose2d[] posesArray = iterationPoses.toArray(new Pose2d[0]);
        double[] distancesArray = iterationDistances.stream().mapToDouble(Double::doubleValue).toArray();
        double[] tofErrorsArray = iterationTofErrors.stream().mapToDouble(Double::doubleValue).toArray();
        Logger.recordOutput("ShotCalculator/SOTF/Iterations", i);
        Logger.recordOutput("ShotCalculator/SOTF/IterationPoses", posesArray);
        Logger.recordOutput("ShotCalculator/SOTF/IterationDistances", distancesArray);
        Logger.recordOutput("ShotCalculator/SOTF/IterationTOFErrors", tofErrorsArray);

        return new ShotParameters(
            Rotation2d.fromRadians(angle).plus(Rotation2d.k180deg),
            flywheelRpm.get(dst), hoodAngle.get(dst)
        );
    }

    ShotParameters calculate(RobotContainer robot);

    public record ShotParameters(Rotation2d turretAngle, double flywheelRpm, double hoodAngle) {}
}
