package frc.robot.autoalign;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.util.AllianceFlipUtil;

import java.util.List;

/**
 * This interface handles the selection of target poses during the auto align process.
 * Inheritors must choose what pose to output given a {@code TargetSelector.Context} instance.
 *
 * @author Avery Gardner
 * @since 2025
 */
public interface TargetSelector {
    TargetSelector EXAMPLE = new NearestTargetSelector(List.of(new Pose2d()));

    /**
     * @return the pose to target for the given context of the situation
     */
    Pose2d getPose(Context context);

    /**
     * @return true if this target selector allows {@code getPose()} to be called each loop cycle
     */
    default boolean isDynamic() { return false; }

    /**
     * Uses choreo utility methods to flip the given pose if on red alliance
     */
    static Pose2d flipIfRed(Pose2d pose) {
        return AllianceFlipUtil.shouldFlip() ? AllianceFlipUtil.flip(pose) : pose;
    }

    record Context(Pose2d robotPose, Translation2d joystickVector) {}
}
