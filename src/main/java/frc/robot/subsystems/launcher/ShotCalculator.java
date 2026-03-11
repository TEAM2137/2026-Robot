package frc.robot.subsystems.launcher;

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
    static final InterpolatingDoubleTreeMap FLYWHEEL_RPM_HUB = InterpolatingDoubleTreeMap.ofEntries(
        Map.entry(1.3165, 1666.0),
        Map.entry(1.5658, 1715.0),
        Map.entry(2.0083, 1775.0),
        Map.entry(2.6245, 1836.0),
        Map.entry(3.0319, 1898.0),
        Map.entry(3.4845, 1975.0),
        Map.entry(3.8919, 2045.0),
        Map.entry(4.3134, 2100.0),
        Map.entry(5.0096, 2195.0),
        Map.entry(5.7282, 2316.0)
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
        Map.entry(4.5000, 1791.0),
        Map.entry(6.9200, 2137.0),
        Map.entry(8.7000, 2380.0),
        Map.entry(12.0000, 2700.0)
    );
    static final InterpolatingDoubleTreeMap HOOD_ANGLE_PASSING = InterpolatingDoubleTreeMap.ofEntries(
        Map.entry(0.0, 26.0)
    );

    static final InterpolatingDoubleTreeMap TOF_LOOKUP = InterpolatingDoubleTreeMap.ofEntries(
        Map.entry(1.5, 0.976),
        Map.entry(2.5, 1.172),
        Map.entry(3.0, 1.328),
        Map.entry(3.5, 1.484),
        Map.entry(4.0, 1.563)
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

        Logger.recordOutput("LookupTables/TargetPos", target);
        Logger.recordOutput("LookupTables/TurretPos", turretPos);
        Logger.recordOutput("LookupTables/Distance", dst);

        return new ShotParameters(
            Rotation2d.fromRadians(theAngle).plus(Rotation2d.k180deg),
            flywheelRpm.get(dst), hoodAngle.get(dst)
        );
    }

    static ShotParameters simpleSOTFShot(Translation2d target, RobotContainer robot, InterpolatingDoubleTreeMap flywheelRpm, InterpolatingDoubleTreeMap hoodAngle) {
        Translation2d turretPos = robot.launcher.getTurret().getFieldSpacePose(robot).getTranslation();

        double timeOfFlight = TOF_LOOKUP.get(target.getDistance(turretPos));
        // if (!SmartDashboard.containsKey("SOTFOffset")) SmartDashboard.putNumber("SOTFOffset", 0);
        // double offsetScalar = SmartDashboard.getNumber("SOTFOffset", 0);
        
        Translation2d newTarget = target.plus(robot.drive.getLinearSpeedsVector().unaryMinus().times(timeOfFlight));

        double dst = newTarget.getDistance(turretPos);
        double dx = newTarget.getX() - turretPos.getX();
        double dy = newTarget.getY() - turretPos.getY();
        
        double theAngle = Math.atan2(dx, dy);

        Logger.recordOutput("LookupTables/TargetPos", newTarget);
        Logger.recordOutput("LookupTables/SOTFTargetPose", new Pose2d(newTarget, new Rotation2d()));
        Logger.recordOutput("LookupTables/TurretPos", turretPos);
        Logger.recordOutput("LookupTables/Distance", dst);

        return new ShotParameters(
            Rotation2d.fromRadians(theAngle).plus(Rotation2d.k180deg),
            flywheelRpm.get(dst), hoodAngle.get(dst)
        );
    }

    ShotParameters calculate(RobotContainer robot);

    public record ShotParameters(Rotation2d turretAngle, double flywheelRpm, double hoodAngle) {}
}
