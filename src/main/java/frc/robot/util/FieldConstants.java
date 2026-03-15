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

    public static final double cornerOffsetX = 1.5; // Adjustable distance from corner
    public static final double cornerOffsetY = 2.0; // Adjustable distance from corner

    public static final Translation2d blueLeftCorner = new Translation2d(cornerOffsetX, fieldHeight - cornerOffsetY);
    public static final Translation2d blueRightCorner = new Translation2d(cornerOffsetX, cornerOffsetY);
    public static final Translation2d redLeftCorner = new Translation2d(fieldWidth - cornerOffsetX, fieldHeight - cornerOffsetY);
    public static final Translation2d redRightCorner = new Translation2d(fieldWidth - cornerOffsetX, cornerOffsetY);

    public static final Translation2d blueTowerLeftPost = new Translation2d(1.016, 4.323824 - (2.0 * 0.0762));
    public static final Translation2d blueTowerRightPost = new Translation2d(1.016, 3.504692 - (2.5 * 0.0762));
    public static final Translation2d redTowerLeftPost = new Translation2d(fieldWidth - 1.016, fieldHeight - 4.323824);
    public static final Translation2d redTowerRightPost = new Translation2d(fieldWidth - 1.016, fieldHeight - 3.504692);

    public static final double hubFromCenterX = 3.6449;
    public static final double hubFromLeftX = 4.6256;
    public static final double bumpWidth = 1.12776;

    public static final Translation2d blueHub = new Translation2d(fieldWidth / 2.0 - hubFromCenterX, fieldHeight / 2.0);
    public static final Translation2d redHub = new Translation2d(fieldWidth / 2.0 + hubFromCenterX, fieldHeight / 2.0);
    
    public static final double allianceZoneX = hubFromLeftX - bumpWidth / 2.0;
    public static final double passingFlipY = fieldHeight / 2.0;

    public static final double noFireZone1Height = bumpWidth * 1.25;
    public static final double noFireZone1Width = noFireZone1Height;

    public static final double midPassY1 = fieldHeight / 2.0 + 1.6;
    public static final double midPassY2 = fieldHeight / 2.0 - 1.6;

    public static final Rectangle2d noFireZone = new Rectangle2d(
        new Translation2d(hubFromLeftX + bumpWidth / 2.0, fieldHeight / 2.0 + noFireZone1Height / 2.0),
        new Translation2d(hubFromLeftX + bumpWidth / 2.0 + noFireZone1Width, fieldHeight / 2.0 - noFireZone1Height / 2.0));

    // this is a no-fire zone in the opposing alliance's side
    // it's unlikely that we'll need this
    
    // public static final double noFireZone2Height = bumpWidth * 1.25;
    // public static final double noFireZone2Width = noFireZone1Height;
    // public static final Rectangle2d noFireZone2 = new Rectangle2d(
    //     new Translation2d(fieldWidth - hubFromLeftX + bumpWidth / 2.0, fieldHeight / 2.0 + noFireZone2Height / 2.0),
    //     new Translation2d(fieldWidth - hubFromLeftX + bumpWidth / 2.0 + noFireZone2Width, fieldHeight / 2.0 - noFireZone2Height / 2.0));

    static {
        Logger.recordOutput("FieldConstants/BlueTowerLeftPost", new Pose2d(blueTowerLeftPost, new Rotation2d()));
        Logger.recordOutput("FieldConstants/BlueTowerRightPost", new Pose2d(blueTowerRightPost, new Rotation2d()));

        Logger.recordOutput("FieldConstants/NoFireZoneBlue", Utils.createRectOutline(noFireZone, false));
        Logger.recordOutput("FieldConstants/NoFireZoneRed", Utils.createRectOutline(noFireZone, true));
    }
}
