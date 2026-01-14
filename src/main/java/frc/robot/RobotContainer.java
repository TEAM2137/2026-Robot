package frc.robot;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.autoalign.AutoAlignCommand;
import frc.robot.commands.DriveCommands;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.FieldConstants;

public class RobotContainer {
    private static RobotContainer instance;

    // Subsystems
    public final Drive drive;
    public final Vision vision;

    // Auto
    public final Autonomous autonomous;

    // Controllers
    private final CommandXboxController driverController = new CommandXboxController(0);
    private final CommandXboxController operatorController = new CommandXboxController(1);
    private final Supplier<Translation2d> joystickSupplier = () -> new Translation2d(driverController.getLeftY(), driverController.getLeftX());

    // Utilities
    public final Trigger resetGyro = driverController.start();

    /** The container for the robot. Contains subsystems, IO devices, and commands. */
    public RobotContainer() {
        RobotContainer.instance = this;

        switch (Constants.currentMode) {
        case REAL:
            // Real robot, instantiate hardware IO implementations
            drive = new Drive(
                new GyroIOPigeon2(),
                new ModuleIOTalonFX(TunerConstants.FrontLeft),
                new ModuleIOTalonFX(TunerConstants.FrontRight),
                new ModuleIOTalonFX(TunerConstants.BackLeft),
                new ModuleIOTalonFX(TunerConstants.BackRight)
            );

            vision = new Vision(
                drive::addVisionMeasurement,
                // new VisionIOLimelight(VisionConstants.cam0, drive::getRotation),
                // new VisionIOLimelight(VisionConstants.cam1, drive::getRotation)
                new VisionIO() {},
                new VisionIO() {}
            );

            break;

        case SIM:
            // Sim robot, instantiate physics sim IO implementations
            drive = new Drive(
                new GyroIO() {},
                new ModuleIOSim(TunerConstants.FrontLeft),
                new ModuleIOSim(TunerConstants.FrontRight),
                new ModuleIOSim(TunerConstants.BackLeft),
                new ModuleIOSim(TunerConstants.BackRight)
            );

            vision = new Vision(
                drive::addVisionMeasurement,
                new VisionIO() {},
                new VisionIO() {}
            );

            break;

        default:
            // Replayed robot, disable IO implementations
            drive = new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {}
            );

            vision = new Vision(
                drive::addVisionMeasurement,
                new VisionIO() {},
                new VisionIO() {}
            );

            break;
        }

        // Setup autonomous features
        autonomous = new Autonomous(this);

        // Configure the controller bindings
        configureControllerBindings();
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureControllerBindings() {
        BooleanSupplier slowMode = () -> driverController.getLeftTriggerAxis() > 0.25;

        // Default command, normal field-relative drive
        drive.setDefaultCommand(DriveCommands.joystickDrive(drive, joystickSupplier,
                        slowMode, () -> -driverController.getRightX() * 0.75)
                .withName("Default Drive"));

        AutoAlignCommand align = new AutoAlignCommand()
                .withTargetPose(new Pose2d(
                        new Translation2d(FieldConstants.FIELD_LENGTH / 2.0, FieldConstants.FIELD_WIDTH / 2.0),
                        new Rotation2d()
                ))
                .withSpeedLimit(10.0);

        driverController.a().whileTrue(align);

        // Reset gyro to 0°
        resetGyro.onTrue(Commands.runOnce(() ->
                drive.setPose(new Pose2d(
                        drive.getPose().getTranslation(),
                        AllianceFlipUtil.shouldFlip()
                            ? AllianceFlipUtil.flip(new Rotation2d())
                            : new Rotation2d()
                )), drive)
                .ignoringDisable(true)
                .withName("Reset Gyro"));
    }

    public Supplier<Translation2d> joystickMotionSupplier() {
        return joystickSupplier;
    }

    public static RobotContainer getInstance() { return instance; }
}
