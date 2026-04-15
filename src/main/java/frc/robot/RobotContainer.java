package frc.robot;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.DriveCommands;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.drive.*;
import frc.robot.subsystems.indexer.*;
import frc.robot.subsystems.intake.*;
import frc.robot.subsystems.launcher.*;
import frc.robot.subsystems.launcher.hood.*;
import frc.robot.subsystems.launcher.turret.*;
import frc.robot.subsystems.launcher.flywheel.*;
import frc.robot.subsystems.vision.*;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.ShiftInfo;
import frc.robot.util.TestMode;

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

    // Auto
    public Autonomous autonomous;

    // Utilities
    private final Supplier<LimitingProfile> limitingProfileSupplier;
    private final Supplier<Translation2d> joystickSupplier;
    private final DoubleSupplier rotationSupplier;

    /** The container for the robot. Contains subsystems, IO devices, and commands. */
    public RobotContainer() {
        RobotContainer.instance = this;

        switch (Constants.currentMode) {
        case REAL:
            // Real robot, instantiate hardware IO implementations
            drive = new Drive(
                new GyroIOPigeon2(),
                // new ModuleIO() {},
                // new ModuleIO() {},
                // new ModuleIO() {},
                // new ModuleIO() {}
                new ModuleIOTalonFX(TunerConstants.FrontLeft),
                new ModuleIOTalonFX(TunerConstants.FrontRight),
                new ModuleIOTalonFX(TunerConstants.BackLeft),
                new ModuleIOTalonFX(TunerConstants.BackRight)
            );

            vision = new Vision(
                drive::addVisionMeasurement,
                new VisionIOPhotonVision("Camera0", VisionConstants.robotToCamera0),
                new VisionIOPhotonVision("Camera1", VisionConstants.robotToCamera1),
                new VisionIOPhotonVision("Camera2", VisionConstants.robotToCamera2),
                new VisionIOPhotonVision("Camera3", VisionConstants.robotToCamera3)
            );

            intake = new Intake(new IntakeIOTalonFX() {});
            indexer = new Indexer(new IndexerIOTalonFX() {});

            launcher = new Launcher(
                new TurretIOTalonFX(),
                new HoodIOTalonFX(),
                new FlywheelIOTalonFX()
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
                new VisionIO() {},
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
                new VisionIO() {},
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
        this.testModeChooser.setDefaultOption("None", TestMode.NONE);
        for (TestMode mode : TestMode.values()) this.testModeChooser.addOption(mode.getName(), mode);
        SmartDashboard.putData("Test Mode", this.testModeChooser);
        if (!SmartDashboard.containsKey("LauncherRPM")) SmartDashboard.putNumber("LauncherRPM", 1000);

        // Setup autonomous features
        this.autonomous = new Autonomous(this);

        // Create utility suppliers
        this.limitingProfileSupplier = () -> launcher.shouldLimitDrive() && !operatorController.x().getAsBoolean() ? LimitingProfile.SOTF : LimitingProfile.DEFAULT;
        this.joystickSupplier = () -> new Translation2d(-driverController.getLeftY(), -driverController.getLeftX());
        this.rotationSupplier = () -> -driverController.getRightX() * 0.75;
        
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
        // Default command, normal field-relative drive
        drive.setDefaultCommand(DriveCommands.joystickDrive(drive, joystickSupplier, rotationSupplier, limitingProfileSupplier)
            .withName("Default Drive Command"));

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
            
        launcher.isLaunching().and(RobotModeTriggers.autonomous().negate()).whileTrue(new SequentialCommandGroup(
            Commands.waitSeconds(Flywheel.Constants.SPIN_UP_TIME),
            new SequentialCommandGroup(
                indexer.run().repeatedly().onlyWhile(launcher.getTurret().isAtTarget()),
                indexer.stop()
            ).repeatedly()
        ).withName("Run Indexer"));

        launcher.isLaunching().and(RobotModeTriggers.autonomous().negate()).whileTrue(Commands.either(
            Commands.none(), intake.agitate(),
            driverController.leftBumper()
        ).withName("Intake Agitation"));

        launcher.isLaunching().and(RobotModeTriggers.autonomous().negate()).onFalse(new SequentialCommandGroup(
            indexer.stop(),
            new ConditionalCommand(
                Commands.none(),
                intake.deploy().andThen(intake.stopRollers()),
                driverController.leftBumper()
            )
        ).withName("Stop Indexer"));

        driverController.leftBumper().onTrue(new SequentialCommandGroup(
            intake.deploy(),
            intake.runRollers()
        ).withName("Intake"));

        driverController.leftBumper().onFalse(new ConditionalCommand(
            intake.agitate(),
            intake.stopIntakeSequence(),
            launcher.isLaunching()
        ).withName("Stop Intaking"));

        this.configureTeleopBindings();
        this.configureTestBindings();
    }

    // configure teleop specific bindings here
    private void configureTeleopBindings() {
        driverController.a().and(RobotModeTriggers.teleop()).whileTrue(DriveCommands.joystickDriveSnake(
            drive, joystickSupplier, limitingProfileSupplier));
        driverController.rightTrigger().and(RobotModeTriggers.teleop()).whileTrue(DriveCommands.joystickDriveCardinalLock(
            drive, joystickSupplier, rotationSupplier, limitingProfileSupplier));
        driverController.leftTrigger().and(RobotModeTriggers.teleop()).whileTrue(DriveCommands.joystickDriveBumpAlign(drive, joystickSupplier));

        driverController.rightBumper().and(RobotModeTriggers.teleop().or(RobotModeTriggers.test()))
            .onTrue(launcher.setState(LaunchState.DONT_LAUNCH).withName("Don't Launch"));

        operatorController.x().and(RobotModeTriggers.teleop()).whileTrue(launcher.setState(LaunchState.DONT_LAUNCH));
        operatorController.x().and(RobotModeTriggers.teleop()).onFalse(launcher.setState(LaunchState.AUTOMATIC));

        driverController.rightBumper().and(RobotModeTriggers.teleop().or(RobotModeTriggers.test()))
            .onFalse(launcher.setState(LaunchState.AUTOMATIC).withName("Re-enable Autofire"));

        Command xLockCommand = drive.xLockCommand().withName("X-Lock");
        Trigger xLock = launcher.isLaunching()
            .and(RobotModeTriggers.teleop())
            .and(() -> this.joystickSupplier.get().getNorm() < DriveCommands.DEADBAND)
            .and(() -> Math.abs(this.driverController.getRightX() * 0.75) < DriveCommands.DEADBAND)
            .and(() -> drive.getDefaultCommand().isScheduled() || xLockCommand.isScheduled())
            .debounce(0.25);

        xLock.whileTrue(xLockCommand);
        
        Command retractIntake = new SequentialCommandGroup(
            intake.retract(),
            intake.runRollers(),
            Commands.waitSeconds(0.5),
            intake.stopRollers()
        ).withName("Retract Intake");

        // RobotModeTriggers.disabled().onFalse(retractIntake);

        operatorController.y().whileTrue(intake.agitate());
        operatorController.y().onFalse(retractIntake);

        operatorController.b().onTrue(intake.deploy().andThen(intake.setRollerVoltage(-12)));
        operatorController.b().onFalse(intake.runRollers());

        operatorController.a().onTrue(indexer.reverse());
        operatorController.a().onFalse(indexer.stop());

        operatorController.rightTrigger(0.98).whileTrue(Commands.runEnd(
            () -> launcher.getHood().setVoltage(-3),
            () -> launcher.getHood().resetPositionRaw()
        ));

        operatorController.leftTrigger(0.98).whileTrue(new SequentialCommandGroup(
            intake.setPivotVoltage(-3),
            Commands.runEnd(() -> {}, () -> intake.resetPosition().schedule())
        ));

        operatorController.povLeft().and(launcher.getTurret().isNotZeroed()).onTrue(launcher.getTurret().setVoltage(-0.5));
        operatorController.povLeft().and(launcher.getTurret().isNotZeroed()).onFalse(launcher.getTurret().setVoltage(0));
        operatorController.povRight().and(launcher.getTurret().isNotZeroed()).onTrue(launcher.getTurret().setVoltage(0.5));
        operatorController.povRight().and(launcher.getTurret().isNotZeroed()).onFalse(launcher.getTurret().setVoltage(0));
        operatorController.start().onTrue(launcher.getTurret().markAsUnzeroed().ignoringDisable(true));

        // operatorController.start().onTrue(launcher.getTurret().resetPosition().ignoringDisable(true));
        operatorController.rightBumper().onTrue(launcher.getHood().resetPosition().ignoringDisable(true));
        operatorController.leftBumper().onTrue(intake.resetPosition().ignoringDisable(true));

        operatorController.povLeft().and(RobotModeTriggers.teleop()).onTrue(launcher.getTurret().increaseTurretOffset());
        operatorController.povRight().and(RobotModeTriggers.teleop()).onTrue(launcher.getTurret().decreaseTurretOffset());
        operatorController.back().and(RobotModeTriggers.teleop()).onTrue(launcher.getTurret().resetTurretOffset());

        // controller rumble (TODO: test this, it doesn't seem to work in sim)
        ShiftInfo.loseAutoTrigger().whileTrue(Commands.runEnd(() -> {
            driverController.setRumble(RumbleType.kBothRumble, 1);
            operatorController.setRumble(RumbleType.kBothRumble, 1);
        }, () -> {
            driverController.setRumble(RumbleType.kBothRumble, 0);
            operatorController.setRumble(RumbleType.kBothRumble, 0);
        }).withTimeout(0.25));
    }

    // configure test mode specific bindings here
    private void configureTestBindings() {
        driverController.povLeft().and(TestMode.TURRET_OFFSETS.isActive()).onTrue(launcher.getTurret().increaseTurretOffset());
        driverController.povRight().and(TestMode.TURRET_OFFSETS.isActive()).onTrue(launcher.getTurret().decreaseTurretOffset());
        driverController.back().and(TestMode.TURRET_OFFSETS.isActive()).onTrue(launcher.getTurret().resetTurretOffset());
        driverController.rightBumper().and(TestMode.TURRET_OFFSETS.isActive()).onTrue(launcher.setState(LaunchState.LAUNCH));
        driverController.rightBumper().and(TestMode.TURRET_OFFSETS.isActive()).onFalse(launcher.setState(LaunchState.AUTOMATIC));

        driverController.povUp().and(TestMode.LOOKUP_TABLES.isActive()).whileTrue(launcher.modifyManualHoodAngle(5));
        driverController.povDown().and(TestMode.LOOKUP_TABLES.isActive()).whileTrue(launcher.modifyManualHoodAngle(-5));
        driverController.povRight().and(TestMode.LOOKUP_TABLES.isActive()).whileTrue(launcher.modifyManualFlywheelRPM(150));
        driverController.povLeft().and(TestMode.LOOKUP_TABLES.isActive()).whileTrue(launcher.modifyManualFlywheelRPM(-150));
        driverController.a().and(TestMode.LOOKUP_TABLES.isActive()).onTrue(indexer.run());
        driverController.a().and(TestMode.LOOKUP_TABLES.isActive()).onFalse(indexer.stop());

        driverController.a().and(TestMode.INTAKE.isActive()).onTrue(intake.deploy());
        driverController.a().and(TestMode.INTAKE.isActive()).onFalse(intake.retract());
        driverController.b().and(TestMode.INTAKE.isActive()).onTrue(intake.runRollers());
        driverController.b().and(TestMode.INTAKE.isActive()).onFalse(intake.stopRollers());

        driverController.b().and(TestMode.HOOD.isActive()).onTrue(launcher.setHoodAngle(20));
        driverController.b().and(TestMode.HOOD.isActive()).onFalse(launcher.setHoodAngle(0));

        driverController.b().and(TestMode.FLYWHEEL.isActive()).onTrue(launcher.setFlywheelSpeed(() -> SmartDashboard.getNumber("LauncherRPM", 1000)));
        driverController.b().and(TestMode.FLYWHEEL.isActive()).onFalse(launcher.setFlywheelVoltage(0));
    }

    public Supplier<Translation2d> joystickMotionSupplier() {
        return joystickSupplier;
    }

    public TestMode getTestMode() {
        if (testModeChooser == null) return TestMode.values()[0];
        return testModeChooser.getSelected();
    }

    public void periodic() {
        ShiftInfo.logShiftInfo();
    }

    public static RobotContainer getInstance() { return instance; }

    public Command stopSubsystems() {
        return new SequentialCommandGroup(
            this.intake.stopRollers().ignoringDisable(true),
            this.indexer.stop().ignoringDisable(true),
            this.launcher.setState(LaunchState.AUTOMATIC)
        );
    }
}
