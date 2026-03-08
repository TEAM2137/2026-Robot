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
        Map.entry(1.3165, 1696.0),
        Map.entry(1.5658, 1744.0),
        Map.entry(2.0083, 1808.0),
        Map.entry(2.6245, 1872.0),
        Map.entry(3.0319, 1934.0),
        Map.entry(3.4845, 2010.0),
        Map.entry(3.8919, 2075.0),
        Map.entry(4.3134, 2120.0)
    );
    static final InterpolatingDoubleTreeMap HOOD_ANGLE_HUB = InterpolatingDoubleTreeMap.ofEntries(
        Map.entry(1.3165, 4.9),
        Map.entry(1.5658, 7.7),
        Map.entry(2.0083, 10.2),
        Map.entry(2.6245, 13.5),
        Map.entry(3.0319, 18.3),
        Map.entry(3.4845, 19.8),
        Map.entry(3.8919, 20.9),
        Map.entry(4.3134, 25.0)
    );

    static final InterpolatingDoubleTreeMap FLYWHEEL_RPM_PASSING = InterpolatingDoubleTreeMap.ofEntries(
        Map.entry(0.0, 2000.0)
    );
    static final InterpolatingDoubleTreeMap HOOD_ANGLE_PASSING = InterpolatingDoubleTreeMap.ofEntries(
        Map.entry(0.0, 25.0)
    );

    static final InterpolatingDoubleTreeMap SOTF_OFFSET_SCALAR = InterpolatingDoubleTreeMap.ofEntries(
        Map.entry(1.5, 5.0),
        Map.entry(2.5, 6.0),
        Map.entry(3.0, 6.8),
        Map.entry(3.5, 7.6),
        Map.entry(4.0, 8.0)
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

        double offsetScalar = SOTF_OFFSET_SCALAR.get(target.getDistance(turretPos));
        // if (!SmartDashboard.containsKey("SOTFOffset")) SmartDashboard.putNumber("SOTFOffset", 0);
        // double offsetScalar = SmartDashboard.getNumber("SOTFOffset", 0);
        
        Translation2d newTarget = target.plus(robot.drive.getLinearSpeedsVector()
            .div(robot.drive.getMaxLinearSpeedMetersPerSec())
            .unaryMinus().times(offsetScalar));

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
