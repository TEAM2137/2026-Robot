package frc.robot.subsystems.drive;

public record LimitingProfile(String name,
    double speedMultiplier, double speedLimitMultiplier, double accelerationLimitMultiplier,
    double omegaMultiplier, double omegaLimitMultiplier, double alphaLimitMultiplier
) {
    public static final LimitingProfile DEFAULT = new LimitingProfile("default",
        1.0, 1.0, 1.0,
        1.0, 1.0, 1.0
    );

    public static final LimitingProfile SOTF = new LimitingProfile("sotf",
        1.0, 0.45, 0.7,
        1.0, 0.2, 0.4
    );

    public static final LimitingProfile BUMP = new LimitingProfile("bump",
        1.0, 0.5, 1.0,
        1.0, 1.0, 1.0
    );
}
