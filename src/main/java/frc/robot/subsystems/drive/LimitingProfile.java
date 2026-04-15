package frc.robot.subsystems.drive;

public record LimitingProfile(
    double speedMultiplier, double speedLimitMultiplier, double accelerationLimitMultiplier,
    double omegaMultiplier, double omegaLimitMultiplier, double alphaLimitMultiplier
) {
    public static final LimitingProfile DEFAULT = new LimitingProfile(
        1.0, 1.0, 1.0,
        1.0, 1.0, 1.0
    );

    public static final LimitingProfile SOTF = new LimitingProfile(
        1.0, 0.4, 0.7,
        1.0, 0.2, 0.4
    );

    public static final LimitingProfile BUMP = new LimitingProfile(
        1.0, 0.6, 1.0,
        1.0, 1.0, 1.0
    );
}
