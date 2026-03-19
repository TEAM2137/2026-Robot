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
    static final int SOTF_MAX_ITERATIONS = 10;
    static final double SOTF_TOF_ERROR_TOLERANCE = 0.01;
    
    static final InterpolatingDoubleTreeMap FLYWHEEL_RPM_HUB = InterpolatingDoubleTreeMap.ofEntries(
        Map.entry(1.172, 1911.0),
        Map.entry(1.741, 1977.0),
        Map.entry(2.224, 1900.0),
        Map.entry(2.484, 1971.0),
        Map.entry(2.801, 2031.0),
        Map.entry(3.236, 2037.0),
        Map.entry(3.794, 2118.0),
        Map.entry(4.261, 2180.0),
        Map.entry(4.655, 2283.0)
    );
    static final InterpolatingDoubleTreeMap HOOD_ANGLE_HUB = InterpolatingDoubleTreeMap.ofEntries(
        Map.entry(1.172, 1.5),
        Map.entry(1.741, 6.2),
        Map.entry(2.224, 12.9),
        Map.entry(2.484, 14.7),
        Map.entry(2.801, 16.8),
        Map.entry(3.236, 22.5),
        Map.entry(3.794, 25.4),
        Map.entry(4.261, 26.0),
        Map.entry(4.655, 26.0)
    );

    static final InterpolatingDoubleTreeMap TIME_OF_FLIGHT_HUB = InterpolatingDoubleTreeMap.ofEntries(
        Map.entry(1.172, 1.17),
        Map.entry(1.741, 1.11),
        Map.entry(2.224, 0.98),
        Map.entry(2.484, 1.02),
        Map.entry(2.801, 1.066),
        Map.entry(3.236, 1.00),
        Map.entry(3.794, 1.06),
        Map.entry(4.261, 1.07),
        Map.entry(4.655, 1.20)
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

    static final ShotCalculator HUB = robot -> {
        Translation2d target = AllianceFlipUtil.either(FieldConstants.blueHub, FieldConstants.redHub);
        return simpleSOTFShot(target, robot, FLYWHEEL_RPM_HUB, HOOD_ANGLE_HUB);
    };

    static final ShotCalculator PASS_LEFT = robot -> {
        Translation2d target = AllianceFlipUtil.either(FieldConstants.blueLeftCorner, FieldConstants.redLeftCorner);
        return simpleSOTFShot(target, robot, FLYWHEEL_RPM_PASSING, HOOD_ANGLE_PASSING);
    };
    static final ShotCalculator PASS_RIGHT = robot -> {
        Translation2d target = AllianceFlipUtil.either(FieldConstants.blueRightCorner, FieldConstants.redRightCorner);
        return simpleSOTFShot(target, robot, FLYWHEEL_RPM_PASSING, HOOD_ANGLE_PASSING);
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
            flywheelRpm.get(dst), hoodAngle.get(dst), TIME_OF_FLIGHT_HUB.get(dst)
        );
    }

    static ShotParameters simpleSOTFShot(Translation2d target, RobotContainer robot, InterpolatingDoubleTreeMap flywheelRpm, InterpolatingDoubleTreeMap hoodAngle) {
        Translation2d turretPos = robot.launcher.getTurret().getFieldSpacePose(robot).getTranslation();
        Translation2d turretVelocity = robot.launcher.getTurret().getFieldSpaceVelocity(robot);

        ArrayList<Pose2d> iterationPoses = new ArrayList<>(SOTF_MAX_ITERATIONS);
        ArrayList<Double> iterationDistances = new ArrayList<>(SOTF_MAX_ITERATIONS);
        ArrayList<Double> iterationTofErrors = new ArrayList<>(SOTF_MAX_ITERATIONS);

        double dst = target.getDistance(turretPos);
        double angle = 0;
        double timeOfFlight = 0;
        double tofError = SOTF_TOF_ERROR_TOLERANCE + 1;
        Translation2d newTarget = target;

        int i;
        for (i = 0; i < SOTF_MAX_ITERATIONS && tofError > SOTF_TOF_ERROR_TOLERANCE; i++) {
            double previousTof = timeOfFlight;
            timeOfFlight = TIME_OF_FLIGHT_HUB.get(dst);
            tofError = Math.abs(previousTof - timeOfFlight);
            
            newTarget = target.minus(turretVelocity.times(timeOfFlight));
            angle = Math.atan2(newTarget.getX() - turretPos.getX(), newTarget.getY() - turretPos.getY());
            dst = newTarget.getDistance(turretPos);

            iterationPoses.add(new Pose2d(newTarget, new Rotation2d()));
            iterationDistances.add(dst);
            iterationTofErrors.add(tofError);
       }

        double finalTof = TIME_OF_FLIGHT_HUB.get(dst);

        Logger.recordOutput("ShotCalculator/SOTF/TargetPose", new Pose2d(newTarget, new Rotation2d()));
        Logger.recordOutput("ShotCalculator/SOTF/Distance", dst);
        Logger.recordOutput("ShotCalculator/SOTF/TimeOfFlight", finalTof);
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
            flywheelRpm.get(dst), hoodAngle.get(dst), finalTof
        );
    }

    ShotParameters calculate(RobotContainer robot);

    public record ShotParameters(Rotation2d turretAngle, double flywheelRpm, double hoodAngle, double timeOfFlight) {}
}
