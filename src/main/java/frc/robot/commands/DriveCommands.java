package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.LimitingProfile;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.FieldConstants;
import frc.robot.util.Utils;

import java.text.DecimalFormat;
import java.text.NumberFormat;
import java.util.LinkedList;
import java.util.List;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

public class DriveCommands {
    public static final double DEADBAND = 0.1;

    public static final double ANGLE_KP = 6.0;
    public static final double ANGLE_KD = 0.2;
    public static final double ANGLE_MAX_VELOCITY = 5.5;
    public static final double ANGLE_MAX_ACCELERATION = 45.0;
    public static final double ANGLE_DEADBAND = 0.0045;

    public static final double FF_START_DELAY = 2.0; // Secs
    public static final double FF_RAMP_RATE = 0.1; // Volts/Sec

    public static final double WHEEL_RADIUS_MAX_VELOCITY = 1.0; // Rad/Sec
    public static final double WHEEL_RADIUS_RAMP_RATE = 0.5; // Rad/Sec^2

    public static final double MAX_LINEAR_ACCELERATION = 10.0; // Meters/Sec^2
    public static final double MAX_ANGULAR_ACCELERATION = 9.0; // Rad/Sec^2

    private DriveCommands() {}

    private static Translation2d getLinearVelocityFromJoysticks(double x, double y) {
        // Apply deadband
        double linearMagnitude = MathUtil.applyDeadband(Math.hypot(x, y), DEADBAND);
        Rotation2d linearDirection = new Rotation2d(Math.atan2(y, x));

        // Square magnitude for more precise control
        linearMagnitude = linearMagnitude * linearMagnitude;

        // Return new linear velocity
        return new Pose2d(new Translation2d(), linearDirection)
            .transformBy(new Transform2d(linearMagnitude, 0.0, new Rotation2d()))
            .getTranslation();
    }

    /**
     * Field relative drive command using two joysticks (controlling linear and angular velocities).
     */
    public static Command joystickDrive(
            Drive drive,
            Supplier<Translation2d> movementSupplier,
            DoubleSupplier omegaSupplier,
            Supplier<LimitingProfile> profileSupplier) {
        return Commands.run(() -> {
            // Get controller drive inputs
            Translation2d movementRaw = movementSupplier.get();

            // Get linear velocity from controller values
            Translation2d linearVelocity = getLinearVelocityFromJoysticks(movementRaw.getX(), movementRaw.getY())
                .times(drive.getMaxLinearSpeedMetersPerSec());

            // Apply rotation deadband
            double omega = MathUtil.applyDeadband(omegaSupplier.getAsDouble(), DEADBAND);

            // Square rotation value for more precise control
            omega = Math.copySign(omega * omega, omega) * drive.getMaxAngularSpeedRadPerSec();

            // Drive the robot
            DriveCommands.driveFieldRelative(drive, linearVelocity, omega, profileSupplier.get());
        }, drive);
    }

    /**
     * Field relative drive command using joystick for linear control and PID for angular control.
     * Possible use cases include snapping to an angle, aiming at a vision target, or controlling
     * absolute rotation with a joystick.
     */
    public static Command joystickDriveAtAngle(
            Drive drive,
            Supplier<Translation2d> movementSupplier,
            Supplier<Rotation2d> rotationSupplier,
            Supplier<LimitingProfile> profileSupplier) {
        // Create PID controller
        ProfiledPIDController angleController =
            new ProfiledPIDController(
                ANGLE_KP,
                0.0,
                ANGLE_KD,
                new TrapezoidProfile.Constraints(ANGLE_MAX_VELOCITY, ANGLE_MAX_ACCELERATION));
        angleController.enableContinuousInput(-Math.PI, Math.PI);

        // Construct command
        return Commands.run(() -> {
            // Get controller drive inputs
            Translation2d movementRaw = movementSupplier.get();

            // Get linear velocity from controller values
            Translation2d linearVelocity = getLinearVelocityFromJoysticks(movementRaw.getX(), movementRaw.getY())
                .times(drive.getMaxLinearSpeedMetersPerSec());

            // Calculate angular velocity from PID
            double omega = angleController.calculate(drive.getRotation().getRadians(), rotationSupplier.get().getRadians());
            if (Math.abs(angleController.getPositionError()) < ANGLE_DEADBAND) omega = 0.0;
            
            // Drive the robot
            DriveCommands.driveFieldRelative(drive, linearVelocity, omega, profileSupplier.get());
        }, drive)
        // Reset PID controller when command starts
        .beforeStarting(() -> angleController.reset(
            drive.getRotation().getRadians(),
            drive.getAngularVelocityRadsPerSec()), drive);
    }

    private static Rotation2d lastRotation = null;

    public static Command joystickDriveSnake(Drive drive, Supplier<Translation2d> movementSupplier, Supplier<LimitingProfile> profileSupplier) {
        return joystickDriveAtAngle(drive, movementSupplier, () -> {
            Translation2d movement = movementSupplier.get();
            if (AllianceFlipUtil.shouldFlip()) movement = AllianceFlipUtil.flip(movement);
            if (lastRotation == null) lastRotation = drive.getRotation();
            if (movement.getNorm() >= DEADBAND) lastRotation = movement.getAngle();
            return lastRotation;
        }, profileSupplier);
    }
    
    public static Command joystickDriveCardinalLock(Drive drive, Supplier<Translation2d> movementSupplier, DoubleSupplier omegaSupplier, Supplier<LimitingProfile> profileSupplier) {
        return joystickDrive(drive, () -> {
            Translation2d movement = movementSupplier.get();
            if (Math.abs(movement.getX()) > Math.abs(movement.getY()))
                return new Translation2d(movement.getX(), 0.0);
            return new Translation2d(0.0, movement.getY());
        }, omegaSupplier, profileSupplier);
    }
    
    public static Command joystickDriveBumpAlign(Drive drive, Supplier<Translation2d> movementSupplier) {
        Supplier<Translation2d> newMovementSupplier = () -> {
            double robotY = drive.getPose().getTranslation().getY();
            double targetY = robotY > FieldConstants.fieldHeight / 2.0
                ? FieldConstants.topBumpCenterY : FieldConstants.bottomBumpCenterY;
            return new Translation2d(
                movementSupplier.get().getX(),
                (targetY - robotY) * 2f * (AllianceFlipUtil.shouldFlip() ? -1 : 1)
            );
        };       
        Supplier<Rotation2d> angleSupplier = () -> {
            Rotation2d rotation = drive.getRotation();
            double angle = MathUtil.inputModulus(rotation.getDegrees(), -180, 180);
            if (angle >= 0) return Rotation2d.fromDegrees((angle < 90) ? 0 : 180);
            else return Rotation2d.fromDegrees((angle > -90) ? 0 : -180);
        };
        return joystickDriveAtAngle(drive, newMovementSupplier, angleSupplier, () -> LimitingProfile.BUMP);
    }

    public static void driveFieldRelative(Drive drive, Translation2d velocity, double omega, LimitingProfile profile) {
        double maxAccel = MAX_LINEAR_ACCELERATION;
        double maxAlpha = MAX_ANGULAR_ACCELERATION;

        Logger.recordOutput("Drive/LimitingProfile", profile.name());

        // Limit linear acceleration
        maxAccel *= profile.accelerationLimitMultiplier();

        // Limit angular acceleration
        maxAlpha *= profile.alphaLimitMultiplier();

        // Limit linear velocity
        velocity = velocity.times(profile.speedMultiplier());
        if (velocity.getNorm() != 0) velocity = Utils.normalize(velocity).times(Math.min(
            velocity.getNorm(), drive.getMaxLinearSpeedMetersPerSec() * profile.speedLimitMultiplier()));

        // Limit angular velocity
        omega *= profile.omegaMultiplier();
        if (omega != 0) omega = MathUtil.clamp(omega, -drive.getMaxAngularSpeedRadPerSec() * profile.omegaLimitMultiplier(),
            drive.getMaxAngularSpeedRadPerSec() * profile.omegaLimitMultiplier());

        // Record commanded velocities
        Logger.recordOutput("Drive/CommandedVelocity", velocity);
        Logger.recordOutput("Drive/CommandedOmega", omega);

        // Apply acceleration limits
        Translation2d finalVelocity = limitAccelerationFor(drive.getLinearSpeedsVector(), velocity, maxAccel);
        double finalOmega = limitAngularAccelerationFor(drive.getAngularVelocityRadsPerSec(), omega, maxAlpha);
        Logger.recordOutput("Drive/FinalLimitedVelocity", finalVelocity);
        Logger.recordOutput("Drive/FinalLimitedOmega", finalOmega);

        // Convert to field relative speeds & send command
        ChassisSpeeds speeds = new ChassisSpeeds(finalVelocity.getX(), finalVelocity.getY(), finalOmega);
        drive.runVelocity(ChassisSpeeds.fromFieldRelativeSpeeds(speeds,
            AllianceFlipUtil.shouldFlip()
                ? drive.getRotation().plus(new Rotation2d(Math.PI))
                : drive.getRotation()));
    }

    /**
     * Measures the velocity feedforward constants for the drive motors.
     *
     * <p>This command should only be used in voltage control mode.
     */
    public static Command feedforwardCharacterization(Drive drive) {
        List<Double> velocitySamples = new LinkedList<>();
        List<Double> voltageSamples = new LinkedList<>();
        Timer timer = new Timer();

        return Commands.sequence(
            // Reset data
            Commands.runOnce(() -> {
                velocitySamples.clear();
                voltageSamples.clear();
            }),

            // Allow modules to orient
            Commands.run(() -> drive.runCharacterization(0.0), drive)
                    .withTimeout(FF_START_DELAY),

            // Start timer
            Commands.runOnce(timer::restart),

            // Accelerate and gather data
            Commands.run(() -> {
                double voltage = timer.get() * FF_RAMP_RATE;
                drive.runCharacterization(voltage);
                velocitySamples.add(drive.getFFCharacterizationVelocity());
                voltageSamples.add(voltage);
            }, drive)
            // When cancelled, calculate and print results
            .finallyDo(() -> {
                int n = velocitySamples.size();
                double sumX = 0.0;
                double sumY = 0.0;
                double sumXY = 0.0;
                double sumX2 = 0.0;
                for (int i = 0; i < n; i++) {
                    sumX += velocitySamples.get(i);
                    sumY += voltageSamples.get(i);
                    sumXY += velocitySamples.get(i) * voltageSamples.get(i);
                    sumX2 += velocitySamples.get(i) * velocitySamples.get(i);
                }
                double kS = (sumY * sumX2 - sumX * sumXY) / (n * sumX2 - sumX * sumX);
                double kV = (n * sumXY - sumX * sumY) / (n * sumX2 - sumX * sumX);

                NumberFormat formatter = new DecimalFormat("#0.00000");
                System.out.println("********** Drive FF Characterization Results **********");
                System.out.println("\tkS: " + formatter.format(kS));
                System.out.println("\tkV: " + formatter.format(kV));
            })
        );
    }

    /** Measures the robot's wheel radius by spinning in a circle. */
    public static Command wheelRadiusCharacterization(Drive drive) {
        SlewRateLimiter limiter = new SlewRateLimiter(WHEEL_RADIUS_RAMP_RATE);
        WheelRadiusCharacterizationState state = new WheelRadiusCharacterizationState();

        return Commands.parallel(
            // Drive control sequence
            Commands.sequence(
                // Reset acceleration limiter
                Commands.runOnce(() -> limiter.reset(0.0)),

                // Turn in place, accelerating up to full speed
                Commands.run(() -> {
                    double speed = limiter.calculate(WHEEL_RADIUS_MAX_VELOCITY);
                    drive.runVelocity(new ChassisSpeeds(0.0, 0.0, speed));
                }, drive)
            ),

            // Measurement sequence
            Commands.sequence(
                // Wait for modules to fully orient before starting measurement
                Commands.waitSeconds(1.0),

                // Record starting measurement
                Commands.runOnce(() -> {
                    state.positions = drive.getWheelRadiusCharacterizationPositions();
                    state.lastAngle = drive.getRotation();
                    state.gyroDelta = 0.0;
                }),

                // Update gyro delta
                Commands.run(() -> {
                    var rotation = drive.getRotation();
                    state.gyroDelta += Math.abs(rotation.minus(state.lastAngle).getRadians());
                    state.lastAngle = rotation;
                })
                // When cancelled, calculate and print results
                .finallyDo(() -> {
                    double[] positions = drive.getWheelRadiusCharacterizationPositions();
                    double wheelDelta = 0.0;
                    for (int i = 0; i < 4; i++) {
                        wheelDelta += Math.abs(positions[i] - state.positions[i]) / 4.0;
                    }
                    double wheelRadius = (state.gyroDelta * Drive.DRIVE_BASE_RADIUS) / wheelDelta;

                    NumberFormat formatter = new DecimalFormat("#0.000");
                    System.out.println("********** Wheel Radius Characterization Results **********");
                    System.out.println("\tWheel Delta: " + formatter.format(wheelDelta) + " radians");
                    System.out.println("\tGyro Delta: " + formatter.format(state.gyroDelta) + " radians");
                    System.out.println("\tWheel Radius: "
                            + formatter.format(wheelRadius)
                            + " meters, "
                            + formatter.format(Units.metersToInches(wheelRadius))
                            + " inches");
                })
            )
        );
    }

    private static class WheelRadiusCharacterizationState {
        double[] positions = new double[4];
        Rotation2d lastAngle = new Rotation2d();
        double gyroDelta = 0.0;
    }

    public static ProfiledPIDController getAngleController() {
        return new ProfiledPIDController(
            ANGLE_KP, 0.0, ANGLE_KD,
            new TrapezoidProfile.Constraints(ANGLE_MAX_VELOCITY, ANGLE_MAX_ACCELERATION)
        );
    }

    public static Translation2d limitAccelerationFor(Translation2d currentVelocity, Translation2d wantedVelocity, double maxAcceleration) {
        Translation2d flippedVelocity = AllianceFlipUtil.shouldFlip() ? new Translation2d().minus(currentVelocity) : currentVelocity;
        Translation2d desiredAccel = wantedVelocity.minus(flippedVelocity);
        double step = maxAcceleration * 0.2;
        if (desiredAccel.getNorm() > step)
            return flippedVelocity.plus(Utils.normalize(desiredAccel).times(step));
        else return wantedVelocity;
    }

    public static double limitAngularAccelerationFor(double currentVelocity, double wantedVelocity, double maxAcceleration) {
        double desiredAccel = wantedVelocity - currentVelocity;
        double step = maxAcceleration * 0.2;
        if (Math.abs(desiredAccel) > step)
            return currentVelocity + Math.signum(desiredAccel) * step;
        else return wantedVelocity;
    }
}
