package frc.robot.subsystems.launcher;

import java.util.Map;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import frc.robot.RobotContainer;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.FieldConstants;

@FunctionalInterface
public interface ShotCalculator {
    static final InterpolatingDoubleTreeMap FLYWHEEL_RPM_HUB = InterpolatingDoubleTreeMap.ofEntries(Map.entry(0.0, 1000.0));
    static final InterpolatingDoubleTreeMap HOOD_ANGLE_HUB = InterpolatingDoubleTreeMap.ofEntries(Map.entry(0.0, 0.0));

    static final ShotCalculator HUB = robot -> {
        Translation2d target = AllianceFlipUtil.either(FieldConstants.blueHub, FieldConstants.redHub);
        return simpleLookupShot(target, robot);
    };
    static final ShotCalculator PASS_LEFT = robot -> {
        Translation2d target = AllianceFlipUtil.either(FieldConstants.blueLeftCorner, FieldConstants.redLeftCorner);
        return simpleLookupShot(target, robot);
    };
    static final ShotCalculator PASS_RIGHT = robot -> {
        Translation2d target = AllianceFlipUtil.either(FieldConstants.blueRightCorner, FieldConstants.redRightCorner);
        return simpleLookupShot(target, robot);
    };

    static ShotParameters simpleLookupShot(Translation2d target, RobotContainer robot) {
        Translation2d turretPos = robot.launcher.getTurret().getFieldSpacePose(robot).getTranslation();

        double dst = target.getDistance(turretPos);
        double dx = target.getX() - turretPos.getX();
        double dy = target.getY() - turretPos.getY();
        
        double theAngle = Math.atan2(dx, dy);

        return new ShotParameters(
            Rotation2d.fromRadians(theAngle).plus(Rotation2d.k180deg),
            FLYWHEEL_RPM_HUB.get(dst),
            HOOD_ANGLE_HUB.get(dst)
        );
    }

    ShotParameters calculate(RobotContainer robot);

    public record ShotParameters(Rotation2d turretAngle, double flywheelRpm, double hoodAngle) {}
}
