package frc.robot.util;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rectangle2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

/** Field constants such as important poses or dimensions can be placed here */
public class FieldConstants {
    public static final double fieldWidth = 16.541;
    public static final double fieldHeight = 8.0692;

    public static final double cornerOffset = 1; /*Adjustable distance from corner*/
    //Left/right is from Driver POV
    public static final Translation2d blueLeftCorner = new Translation2d(cornerOffset, fieldHeight - cornerOffset);
    public static final Translation2d blueRightCorner = new Translation2d(cornerOffset, cornerOffset);
    public static final Translation2d redLeftCorner = new Translation2d(fieldWidth - cornerOffset, fieldHeight - cornerOffset);
    public static final Translation2d redRightCorner = new Translation2d(fieldWidth - cornerOffset, cornerOffset);

    public static final Translation2d blueTowerLeftPost = new Translation2d(1.016, 4.323824);
    public static final Translation2d blueTowerRightPost = new Translation2d(1.016, 3.504692);
    public static final Translation2d redTowerLeftPost = new Translation2d(fieldWidth - 1.016, fieldHeight - 4.323824);
    public static final Translation2d redTowerRightPost = new Translation2d(fieldWidth - 1.016, fieldHeight - 3.504692);


    public static final double hubFromCenterX = 3.6449;
    public static final double hubFromLeftX = 4.6256;
    public static final double bumpWidth = 1.12776;

    public static final Translation2d blueHub = new Translation2d(fieldWidth / 2.0 - hubFromCenterX, fieldHeight / 2.0);
    public static final Translation2d redHub = new Translation2d(fieldWidth / 2.0 + hubFromCenterX, fieldHeight / 2.0);
    
    public static final double allianceZoneX = hubFromLeftX - bumpWidth / 2.0;
    public static final double passingFlipY = fieldHeight / 2.0;

    // TODO: tune width and height
    public static final double noFireZone1Height = bumpWidth;
    public static final double noFireZone1Width = noFireZone1Height;
    public static final Rectangle2d noFireZone1 = new Rectangle2d(
        new Translation2d(hubFromLeftX + bumpWidth / 2.0, fieldHeight / 2.0 + noFireZone1Height / 2.0),
        new Translation2d(hubFromLeftX + bumpWidth / 2.0 + noFireZone1Width, fieldHeight / 2.0 - noFireZone1Height / 2.0));

    // TODO: tune width and height
    public static final double noFireZone2Height = bumpWidth;
    public static final double noFireZone2Width = noFireZone1Height;
    public static final Rectangle2d noFireZone2 = new Rectangle2d(
        new Translation2d(fieldWidth - hubFromLeftX + bumpWidth / 2.0, fieldHeight / 2.0 + noFireZone2Height / 2.0),
        new Translation2d(fieldWidth - hubFromLeftX + bumpWidth / 2.0 + noFireZone2Width, fieldHeight / 2.0 - noFireZone2Height / 2.0));

    static {
        Logger.recordOutput("Climb/BlueTowerLeftPost", new Pose2d(blueTowerLeftPost, new Rotation2d()));
        Logger.recordOutput("Climb/BlueTowerRightPost", new Pose2d(blueTowerRightPost, new Rotation2d()));

    }
}
