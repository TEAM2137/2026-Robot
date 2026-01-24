package frc.robot.subsystems.launcher;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.RobotContainer;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.FieldConstants;

public class HubShotCalculator implements ShotCalculator {
    @Override
    public ShotParameters calculate(RobotContainer robot) {
        Translation2d target = AllianceFlipUtil.either(FieldConstants.blueHub, FieldConstants.redHub);
        Translation2d robotPos = robot.drive.getPose().getTranslation();

        double dst = target.getDistance(robotPos);
        double dx = target.getX() - robotPos.getX();
        double dy = target.getY() - robotPos.getY();
        
        double theAngle = Math.atan2(dy, dx);

        return new ShotParameters(
            Rotation2d.fromRadians(theAngle),
            robot.launcher.getRpm(dst),
            robot.launcher.getHoodAngle(dst)
        );
    }
}
