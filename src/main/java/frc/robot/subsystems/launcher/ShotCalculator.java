package frc.robot.subsystems.launcher;

import java.util.ArrayList;

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
    static final ShotCalculator HUB = robot -> {
        Translation2d target = AllianceFlipUtil.either(FieldConstants.blueHub, FieldConstants.redHub);
        return simpleSOTFShot(target, robot,
            LookupTables.flywheelRpmHub,
            LookupTables.hoodAngleHub, 
            LookupTables.timeOfFlightHub
        );
    };

    static final ShotCalculator PASS_LEFT = robot -> {
        Translation2d target = AllianceFlipUtil.either(FieldConstants.blueLeftCorner, FieldConstants.redLeftCorner);
        return simpleSOTFShot(target, robot,
            LookupTables.flywheelRpmPassing,
            LookupTables.hoodAnglePassing, 
            LookupTables.timeOfFlightPassing
        );
    };
    static final ShotCalculator PASS_RIGHT = robot -> {
        Translation2d target = AllianceFlipUtil.either(FieldConstants.blueRightCorner, FieldConstants.redRightCorner);
        return simpleSOTFShot(target, robot,
            LookupTables.flywheelRpmPassing,
            LookupTables.hoodAnglePassing, 
            LookupTables.timeOfFlightPassing
        );
    };

    static final ShotCalculator DYNAMIC_PASSING = robot -> {
        Pose2d pose = robot.drive.getPose();
        Translation2d target;
        if (pose.getY() < FieldConstants.passingMidZoneY1) target = FieldConstants.leftPassTarget;
        else if (pose.getY() > FieldConstants.passingMidZoneY2) target = FieldConstants.rightPassTarget;
        else return simpleSOTFShot(
            AllianceFlipUtil.shouldFlip()
                ? AllianceFlipUtil.flip(FieldConstants.midPassTarget)
                : FieldConstants.midPassTarget, robot,
            LookupTables.flywheelRpmPassing,
            LookupTables.hoodAnglePassing,
            LookupTables.timeOfFlightPassing
        );
        double offset = (target.getY() - pose.getY()) * 0.5;
        double flippedX = AllianceFlipUtil.shouldFlip() ? AllianceFlipUtil.flipX(target.getX()) : target.getX();
        return simpleSOTFShot(
            new Translation2d(flippedX, target.getY() + offset), robot,
            LookupTables.flywheelRpmPassing,
            LookupTables.hoodAnglePassing,
            LookupTables.timeOfFlightPassing
        );
    };
    
    static final int SOTF_MAX_ITERATIONS = 10;
    static final double SOTF_TOF_ERROR_TOLERANCE = 0.01;

    static ShotParameters simpleLookupShot(Translation2d target, RobotContainer robot,
            InterpolatingDoubleTreeMap flywheelRpm, InterpolatingDoubleTreeMap hoodAngle, InterpolatingDoubleTreeMap tofLookup) {
        Translation2d turretPos = robot.launcher.getTurret().getFieldSpacePose(robot).getTranslation();

        double dst = target.getDistance(turretPos);
        double dx = target.getX() - turretPos.getX();
        double dy = target.getY() - turretPos.getY();
        
        double theAngle = Math.atan2(dx, dy);

        Logger.recordOutput("ShotCalculator/TargetPos", target);
        Logger.recordOutput("ShotCalculator/TurretPos", turretPos);
        Logger.recordOutput("ShotCalculator/Distance", dst);

        return new ShotParameters(
            Rotation2d.fromRadians(theAngle).plus(Rotation2d.k180deg),
            flywheelRpm.get(dst), hoodAngle.get(dst), tofLookup.get(dst)
        );
    }

    static ShotParameters simpleSOTFShot(Translation2d target, RobotContainer robot,
            InterpolatingDoubleTreeMap flywheelRpm, InterpolatingDoubleTreeMap hoodAngle, InterpolatingDoubleTreeMap tofLookup) {
        Translation2d turretPos = robot.launcher.getTurret().getFieldSpacePose(robot).getTranslation();
        Translation2d turretVelocity = robot.launcher.getTurret().getFieldSpaceVelocity(robot);

        ArrayList<Pose2d> iterationPoses = new ArrayList<>(SOTF_MAX_ITERATIONS);
        ArrayList<Double> iterationDistances = new ArrayList<>(SOTF_MAX_ITERATIONS);
        ArrayList<Double> iterationTofErrors = new ArrayList<>(SOTF_MAX_ITERATIONS);

        double dst = target.getDistance(turretPos);
        double angle = 0;
        double timeOfFlight = 0;
        double tofError = SOTF_TOF_ERROR_TOLERANCE + 1;
        Translation2d newTarget = target;

        int i;
        for (i = 0; i < SOTF_MAX_ITERATIONS && tofError > SOTF_TOF_ERROR_TOLERANCE; i++) {
            double previousTof = timeOfFlight;
            timeOfFlight = tofLookup.get(dst);
            tofError = Math.abs(previousTof - timeOfFlight);
            
            newTarget = target.minus(turretVelocity.times(timeOfFlight));
            angle = Math.atan2(newTarget.getX() - turretPos.getX(), newTarget.getY() - turretPos.getY());
            dst = newTarget.getDistance(turretPos);

            iterationPoses.add(new Pose2d(newTarget, new Rotation2d()));
            iterationDistances.add(dst);
            iterationTofErrors.add(tofError);
       }

        double finalTof = tofLookup.get(dst);

        Logger.recordOutput("ShotCalculator/SOTF/TargetPose", new Pose2d(newTarget, new Rotation2d()));
        Logger.recordOutput("ShotCalculator/SOTF/Distance", dst);
        Logger.recordOutput("ShotCalculator/SOTF/TimeOfFlight", finalTof);
        Logger.recordOutput("ShotCalculator/SOTF/TurretVelocity", turretVelocity);
        
        Pose2d[] posesArray = iterationPoses.toArray(new Pose2d[0]);
        double[] distancesArray = iterationDistances.stream().mapToDouble(Double::doubleValue).toArray();
        double[] tofErrorsArray = iterationTofErrors.stream().mapToDouble(Double::doubleValue).toArray();
        Logger.recordOutput("ShotCalculator/SOTF/Iterations", i);
        Logger.recordOutput("ShotCalculator/SOTF/IterationPoses", posesArray);
        Logger.recordOutput("ShotCalculator/SOTF/IterationDistances", distancesArray);
        Logger.recordOutput("ShotCalculator/SOTF/IterationTOFErrors", tofErrorsArray);

        return new ShotParameters(
            Rotation2d.fromRadians(angle).plus(Rotation2d.k180deg),
            flywheelRpm.get(dst), hoodAngle.get(dst), finalTof
        );
    }

    ShotParameters calculate(RobotContainer robot);

    public record ShotParameters(Rotation2d turretAngle, double flywheelRpm, double hoodAngle, double timeOfFlight) {}
}
