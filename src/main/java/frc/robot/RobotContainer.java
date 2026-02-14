package frc.robot;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.commands.DriveCommands;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.indexer.IndexerIO;
import frc.robot.subsystems.indexer.IndexerIOSim;
import frc.robot.subsystems.indexer.IndexerIOTalonFX;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.intake.IntakeIOSim;
import frc.robot.subsystems.intake.IntakeIOTalonFX;
import frc.robot.subsystems.launcher.Launcher;
import frc.robot.subsystems.launcher.flywheel.FlywheelIO;
import frc.robot.subsystems.launcher.flywheel.FlywheelIOSim;
import frc.robot.subsystems.launcher.flywheel.FlywheelIOTalonFX;
import frc.robot.subsystems.launcher.hood.HoodIO;
import frc.robot.subsystems.launcher.hood.HoodIOSim;
import frc.robot.subsystems.launcher.turret.TurretIO;
import frc.robot.subsystems.launcher.turret.TurretIOSim;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.TestMode;
import frc.robot.util.Utils;
import frc.robot.util.Utils.MatchEvent;

public class RobotContainer {
    private static RobotContainer instance;
    
    // Test modes
    public final SendableChooser<TestMode> testModeChooser;

    // Subsystems
    public final Drive drive;
    public final Vision vision;
    public final Intake intake;
    public final Indexer indexer;
    public final Launcher launcher;

    // Controllers
    public final CommandXboxController driverController = new CommandXboxController(0);
    public final CommandXboxController operatorController = new CommandXboxController(1);
    private final Supplier<Translation2d> joystickSupplier = () -> new Translation2d(driverController.getLeftY(), driverController.getLeftX());

    // Auto
    public Autonomous autonomous;

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
                new VisionIO() {},
                new VisionIO() {}
            );

            intake = new Intake(new IntakeIOTalonFX());
            indexer = new Indexer(new IndexerIOTalonFX() {});

            launcher = new Launcher(
                new TurretIO() {},
                new HoodIO() {},
                new FlywheelIOTalonFX() {}
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

            intake = new Intake(new IntakeIOSim() {});
            indexer = new Indexer(new IndexerIOSim() {});

            launcher = new Launcher(
                new TurretIOSim() {},
                new HoodIOSim() {},
                new FlywheelIOSim() {}
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

            intake = new Intake(new IntakeIO() {});
            indexer = new Indexer(new IndexerIO() {});

            launcher = new Launcher(
                new TurretIO() {},
                new HoodIO() {},
                new FlywheelIO() {}
            );

            break;
        }

        // Setup test mode chooser
        this.testModeChooser = new SendableChooser<>();
        this.testModeChooser.setDefaultOption("All", TestMode.ALL);
        for (TestMode mode : TestMode.values()) this.testModeChooser.addOption(mode.getName(), mode);
        SmartDashboard.putData("Test Mode", this.testModeChooser);
        SmartDashboard.putNumber("LauncherVolts", 5);

        // Setup autonomous features
        this.autonomous = new Autonomous(this);
        
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

        // Reset gyro to 0°
        driverController.start().onTrue(Commands.runOnce(() ->
                drive.setPose(new Pose2d(
                        drive.getPose().getTranslation(),
                        AllianceFlipUtil.shouldFlip()
                            ? AllianceFlipUtil.flip(new Rotation2d())
                            : new Rotation2d()
                )), drive)
                .ignoringDisable(true)
                .withName("Reset Gyro"));

        configureTeleopBindings();
        configureTestBindings();
    }

    // configure teleop specific bindings here
    private void configureTeleopBindings() {
        driverController.rightBumper().and(RobotModeTriggers.teleop()).toggleOnTrue(Commands.runEnd(
            () -> launcher.setIsLaunching(true),
            () -> launcher.setIsLaunching(false)
        ));

        launcher.isLaunching().and(RobotModeTriggers.teleop()).whileTrue(Commands.waitSeconds(0.5)
            .andThen(new SequentialCommandGroup(
                indexer.run().repeatedly().onlyWhile(launcher.getTurret().isAtTarget()),
                indexer.stop()
            )
            .repeatedly()
        ));

        launcher.isLaunching().and(RobotModeTriggers.teleop()).onFalse(indexer.stop());

        driverController.povLeft().whileTrue(launcher.getTurret().increaseTurretOffset());
        driverController.povRight().whileTrue(launcher.getTurret().decreaseTurretOffset());
        driverController.povDown().onTrue(launcher.getTurret().resetTurretOffset());

        driverController.leftBumper().and(RobotModeTriggers.teleop()).onTrue(intake.startIntakeSequence());
        driverController.leftBumper().and(RobotModeTriggers.teleop()).onFalse(intake.stopIntakeSequence());
    }

    // configure test mode specific bindings here
    private void configureTestBindings() {
        driverController.rightBumper().and(TestMode.ALL.isActive()).onTrue(intake.startIntakeSequence());
        driverController.rightBumper().and(TestMode.ALL.isActive()).onFalse(intake.stopIntakeSequence());
        driverController.a().and(TestMode.ALL.isActive()).onTrue(indexer.run());
        driverController.a().and(TestMode.ALL.isActive()).onFalse(indexer.stop());
        driverController.b().and(TestMode.ALL.isActive()).onTrue(launcher.setFlywheelVoltage(() -> SmartDashboard.getNumber("LauncherVolts", 5)));
        driverController.b().and(TestMode.ALL.isActive()).onFalse(launcher.setFlywheelVoltage(() -> 0));

        driverController.povUp().and(TestMode.LOOKUP_TABLES.isActive()).whileTrue(launcher.modifyManualHoodAngle(5));
        driverController.povDown().and(TestMode.LOOKUP_TABLES.isActive()).whileTrue(launcher.modifyManualHoodAngle(-5));
        driverController.povRight().and(TestMode.LOOKUP_TABLES.isActive()).whileTrue(launcher.modifyManualFlywheelRPM(800));
        driverController.povLeft().and(TestMode.LOOKUP_TABLES.isActive()).whileTrue(launcher.modifyManualFlywheelRPM(-800));
    }

    public Supplier<Translation2d> joystickMotionSupplier() {
        return joystickSupplier;
    }

    public TestMode getTestMode() {
        if (testModeChooser == null) return TestMode.values()[0];
        return testModeChooser.getSelected();
    }

    public static void periodic() {
        MatchEvent next = Utils.getNextMatchEvent();
        SmartDashboard.putString("NextMatchEvent", next.toString());
        SmartDashboard.putBoolean("IsHubActive", next.isHubActive());
    }

    public static RobotContainer getInstance() { return instance; }
}
