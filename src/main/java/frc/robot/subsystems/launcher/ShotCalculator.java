package frc.robot.subsystems.launcher;

import java.util.Map;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.RobotContainer;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.FieldConstants;
import frc.robot.util.LookupTable;

@FunctionalInterface
public interface ShotCalculator {
    static final LookupTable FLYWHEEL_RPM_HUB = new LookupTable(Map.of(0.0, 0.0));
    static final LookupTable HOOD_ANGLE_HUB = new LookupTable(Map.of(0.0, 0.0));

    static final ShotCalculator HUB = robot -> {
        Translation2d target = AllianceFlipUtil.either(FieldConstants.blueHub, FieldConstants.redHub);
        return simpleLookupShot(target, robot);
    };

    static ShotParameters simpleLookupShot(Translation2d target, RobotContainer robot) {
        Translation2d robotPos = robot.drive.getPose().getTranslation();

        double dst = target.getDistance(robotPos);
        double dx = target.getX() - robotPos.getX();
        double dy = target.getY() - robotPos.getY();
        
        double theAngle = Math.atan2(dy, dx);

        return new ShotParameters(
            Rotation2d.fromRadians(theAngle),
            FLYWHEEL_RPM_HUB.lookup(dst),
            HOOD_ANGLE_HUB.lookup(dst)
        );
    }

    ShotParameters calculate(RobotContainer robot);

    public record ShotParameters(Rotation2d turretAngle, double flywheelRpm, double hoodAngle) {}
}
