package frc.robot;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.Alert.AlertType;
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
import frc.robot.util.Alerts;
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
    private final Supplier<Translation2d> joystickSupplier = () -> new Translation2d(-driverController.getLeftY(), -driverController.getLeftX());

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

            intake = new Intake(new IntakeIO() {});
            indexer = new Indexer(new IndexerIO() {});

            launcher = new Launcher(3
                new TurretIO() {},
                new HoodIO() {},
                new FlywheelIO() {}
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
        this.testModeChooser.setDefaultOption("All", TestMode.ALL);
        for (TestMode mode : TestMode.values()) this.testModeChooser.addOption(mode.getName(), mode);
        SmartDashboard.putData("Test Mode", this.testModeChooser);
        if (!SmartDashboard.containsKey("LauncherRPM")) SmartDashboard.putNumber("LauncherRPM", 1000);

        // Setup autonomous features
        this.autonomous = new Autonomous(this);
        
        // Configure the controller bindings
        configureControllerBindings();

        // necessary
        Alerts.add("Robot is too tall to fit under the trench!", AlertType.kWarning, () -> DriverStation.isDisabled());
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureControllerBindings() {
        // Default command, normal field-relative drive
        drive.setDefaultCommand(DriveCommands.joystickDrive(drive, joystickSupplier,
                        () -> false, () -> -driverController.getRightX() * 0.75)
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

        this.configureTeleopBindings();
        this.configureTestBindings();
    }

    // configure teleop specific bindings here
    private void configureTeleopBindings() {
        driverController.a().and(RobotModeTriggers.teleop()).whileTrue(DriveCommands.joystickDriveFrontFirst(drive, joystickSupplier));
        driverController.rightTrigger().and(RobotModeTriggers.teleop()).whileTrue(DriveCommands.joystickDriveCardinalDirections(drive, joystickSupplier, () -> -driverController.getRightX() * 0.75));

        driverController.rightBumper().and(RobotModeTriggers.teleop().or(RobotModeTriggers.test()))
            .onTrue(launcher.setState(LaunchState.DONT_LAUNCH).withName("Don't Launch"));

        driverController.rightBumper().and(RobotModeTriggers.teleop().or(RobotModeTriggers.test()))
            .onFalse(launcher.setState(LaunchState.AUTOMATIC).withName("Re-enable Autofire"));

        launcher.isLaunching().and(RobotModeTriggers.teleop()).whileTrue(new SequentialCommandGroup(
            Commands.waitSeconds(Flywheel.Constants.SPIN_UP_TIME),
            new SequentialCommandGroup(
                indexer.run().repeatedly().onlyWhile(launcher.getTurret().isAtTarget()),
                indexer.stop()
            ).repeatedly()
        ).withName("Run Indexer"));

        launcher.isLaunching().and(RobotModeTriggers.teleop()).whileTrue(Commands.either(
            Commands.none(), intake.agitate(),
            driverController.leftBumper()
        ));

        launcher.isLaunching().and(RobotModeTriggers.teleop()).onFalse(new SequentialCommandGroup(
            indexer.stop(),
            new ConditionalCommand(
                Commands.none(),
                intake.deploy().andThen(intake.stopRollers()),
                driverController.leftBumper()
            )
        ).withName("Stop Indexer"));

        Trigger xLock = launcher.isLaunching()
            .and(() -> this.joystickSupplier.get().getNorm() < DriveCommands.DEADBAND)
            .and(() -> Math.abs(this.driverController.getRightX() * 0.75) < DriveCommands.DEADBAND)
            .and(RobotModeTriggers.teleop())
            .debounce(0.25);

        xLock.whileTrue(drive.xLockCommand().withName("X-Lock"));

        driverController.leftBumper().and(RobotModeTriggers.teleop()).onTrue(new SequentialCommandGroup(
            intake.deploy(),
            intake.runRollers()
        ).withName("Intake"));

        driverController.leftBumper().and(RobotModeTriggers.teleop()).onFalse(new ConditionalCommand(
            intake.agitate(),
            intake.stopIntakeSequence(),
            launcher.isLaunching()
        ).withName("Stop Intaking"));
        
        Command retractIntake = new SequentialCommandGroup(
            intake.retract(),
            intake.runRollers(),
            Commands.waitSeconds(0.5),
            intake.stopRollers()
        ).withName("Retract Intake");

        // RobotModeTriggers.disabled().onFalse(retractIntake);

        driverController.leftTrigger().and(RobotModeTriggers.teleop()).whileTrue(intake.agitate());
        driverController.leftTrigger().and(RobotModeTriggers.teleop()).onFalse(retractIntake);

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
    }

    // configure test mode specific bindings here
    private void configureTestBindings() {
        driverController.leftBumper().and(TestMode.ALL.isActive()).onTrue(intake.startIntakeSequence());
        driverController.leftBumper().and(TestMode.ALL.isActive()).onFalse(intake.stopIntakeSequence());
        driverController.a().and(TestMode.ALL.isActive()).onTrue(indexer.run().andThen(intake.agitate()));
        driverController.a().and(TestMode.ALL.isActive()).onFalse(indexer.stop().andThen(intake.retract().andThen(intake.stopRollers())));
        driverController.b().and(TestMode.ALL.isActive()).onTrue(launcher.setFlywheelSpeed(() -> SmartDashboard.getNumber("LauncherRPM", 1000)));
        driverController.b().and(TestMode.ALL.isActive()).onFalse(launcher.setFlywheelVoltage(() -> 0));
        driverController.y().and(TestMode.ALL.isActive()).onTrue(launcher.setHoodAngle(18));
        driverController.x().and(TestMode.ALL.isActive()).onTrue(launcher.setHoodAngle(0));

        driverController.leftBumper().and(TestMode.INTAKE.isActive()).onTrue(intake.deploy());
        driverController.leftBumper().and(TestMode.INTAKE.isActive()).onFalse(intake.retract());
        driverController.b().and(TestMode.INTAKE.isActive()).onTrue(intake.runRollers());
        driverController.b().and(TestMode.INTAKE.isActive()).onFalse(intake.stopRollers());

        driverController.b().and(TestMode.HOOD.isActive()).onTrue(launcher.setHoodAngle(18));
        driverController.b().and(TestMode.HOOD.isActive()).onFalse(launcher.setHoodAngle(0));

        driverController.b().and(TestMode.FLYWHEEL.isActive()).onTrue(launcher.setFlywheelSpeed(() -> SmartDashboard.getNumber("LauncherRPM", 1000)));
        driverController.b().and(TestMode.FLYWHEEL.isActive()).onFalse(launcher.setFlywheelVoltage(0));
        
        driverController.povUp().and(TestMode.LOOKUP_TABLES.isActive()).whileTrue(launcher.modifyManualHoodAngle(5));
        driverController.povDown().and(TestMode.LOOKUP_TABLES.isActive()).whileTrue(launcher.modifyManualHoodAngle(-5));
        driverController.povRight().and(TestMode.LOOKUP_TABLES.isActive()).whileTrue(launcher.modifyManualFlywheelRPM(150));
        driverController.povLeft().and(TestMode.LOOKUP_TABLES.isActive()).whileTrue(launcher.modifyManualFlywheelRPM(-150));
        driverController.a().and(TestMode.LOOKUP_TABLES.isActive()).onTrue(indexer.run());
        driverController.a().and(TestMode.LOOKUP_TABLES.isActive()).onFalse(indexer.stop());
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
