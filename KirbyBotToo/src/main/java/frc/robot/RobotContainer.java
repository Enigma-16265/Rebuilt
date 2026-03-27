package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;

import java.util.Optional;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.Driving;
import frc.robot.commands.AutoRoutines;
import frc.robot.commands.ManualDriveCommand;
import frc.robot.commands.SubsystemCommands;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Floor;
import frc.robot.subsystems.Hanger;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;
import frc.util.SwerveTelemetry;

/**
 * This class is where the bulk of the robot should be declared.
 */
public class RobotContainer {
    private enum DriverControllerType {
        XBOX,
        PS5
    }

    // Change this one constant at build time.
    private static final DriverControllerType DRIVER_CONTROLLER_TYPE = DriverControllerType.XBOX;
    private static final int DRIVER_CONTROLLER_PORT = 0;

    private final Swerve swerve = TunerConstants.createDrivetrain();
    private final Intake intake = new Intake();
    private final Floor floor = new Floor();
    private final Feeder feeder = new Feeder();
    private final Shooter shooter = new Shooter();
    private final Hood hood = new Hood();
    private final Hanger hanger = new Hanger();
    private final Limelight limelight = new Limelight("limelight");

    private final SwerveTelemetry swerveTelemetry =
        new SwerveTelemetry(Driving.kMaxSpeed.in(MetersPerSecond));

    private final DriverControls driver = createDriverControls();

    private final AutoRoutines autoRoutines = new AutoRoutines(
        swerve,
        intake,
        floor,
        feeder,
        shooter,
        hood,
        hanger,
        limelight
    );

    private final SubsystemCommands subsystemCommands = new SubsystemCommands(
        swerve,
        intake,
        floor,
        feeder,
        shooter,
        hood,
        hanger,
        () -> -driver.getLeftY(),
        () -> -driver.getLeftX()
    );

    public RobotContainer() {
        configureBindings();
        autoRoutines.configure();
        swerve.registerTelemetry(swerveTelemetry::telemeterize);
    }

    private DriverControls createDriverControls() {
        return switch (DRIVER_CONTROLLER_TYPE) {
            case XBOX -> new XboxDriverControls(DRIVER_CONTROLLER_PORT);
            case PS5 -> new Ps5DriverControls(DRIVER_CONTROLLER_PORT);
        };
    }

    private void configureBindings() {
        configureManualDriveBindings();
        limelight.setDefaultCommand(updateVisionCommand());

        RobotModeTriggers.autonomous().or(RobotModeTriggers.teleop())
            .onTrue(intake.homingCommand())
            .onTrue(hanger.homingCommand());

        // Same physical controls on both controllers:
        // Right trigger / R2
        driver.rightTrigger().whileTrue(subsystemCommands.aimAndShoot());

        // Right bumper / R1
        driver.rightBumper().whileTrue(subsystemCommands.shootManually());

        // Left trigger / L2
        driver.leftTrigger().whileTrue(intake.intakeCommand());

        // Left bumper / L1
        driver.leftBumper().onTrue(intake.runOnce(() -> intake.set(Intake.Position.STOWED)));

        driver.povUp().onTrue(hanger.positionCommand(Hanger.Position.HANGING));
        driver.povDown().onTrue(hanger.positionCommand(Hanger.Position.HUNG));
    }

    private void configureManualDriveBindings() {
        final ManualDriveCommand manualDriveCommand = new ManualDriveCommand(
            swerve,
            () -> -driver.getLeftY(),
            () -> -driver.getLeftX(),
            () -> -driver.getRightX()
        );

        swerve.setDefaultCommand(manualDriveCommand);

        // Face buttons kept in the same physical locations:
        // Bottom: A / Cross
        driver.faceDown().onTrue(
            Commands.runOnce(() -> manualDriveCommand.setLockedHeading(Rotation2d.k180deg))
        );

        // Right: B / Circle
        driver.faceRight().onTrue(
            Commands.runOnce(() -> manualDriveCommand.setLockedHeading(Rotation2d.kCW_90deg))
        );

        // Left: X / Square
        driver.faceLeft().onTrue(
            Commands.runOnce(() -> manualDriveCommand.setLockedHeading(Rotation2d.kCCW_90deg))
        );

        // Top: Y / Triangle
        driver.faceUp().onTrue(
            Commands.runOnce(() -> manualDriveCommand.setLockedHeading(Rotation2d.kZero))
        );

        // Back / Create
        driver.menuLeft().onTrue(
            Commands.runOnce(() -> manualDriveCommand.seedFieldCentric())
        );
    }

    private Command updateVisionCommand() {
        return limelight.run(() -> {
            final Pose2d currentRobotPose = swerve.getState().Pose;
            final Optional<Limelight.Measurement> measurement =
                limelight.getMeasurement(currentRobotPose);

            measurement.ifPresent(m -> {
                swerve.addVisionMeasurement(
                    m.poseEstimate.pose,
                    m.poseEstimate.timestampSeconds,
                    m.standardDeviations
                );
            });
        }).ignoringDisable(true);
    }

    private interface DriverControls {
        double getLeftX();
        double getLeftY();
        double getRightX();

        Trigger leftTrigger();
        Trigger rightTrigger();
        Trigger leftBumper();
        Trigger rightBumper();

        Trigger povUp();
        Trigger povDown();

        Trigger faceDown();   // A / Cross
        Trigger faceRight();  // B / Circle
        Trigger faceLeft();   // X / Square
        Trigger faceUp();     // Y / Triangle

        Trigger menuLeft();   // Back / Create
    }

    private static final class XboxDriverControls implements DriverControls {
        private final CommandXboxController controller;

        private XboxDriverControls(int port) {
            controller = new CommandXboxController(port);
        }

        @Override
        public double getLeftX() {
            return controller.getLeftX();
        }

        @Override
        public double getLeftY() {
            return controller.getLeftY();
        }

        @Override
        public double getRightX() {
            return controller.getRightX();
        }

        @Override
        public Trigger leftTrigger() {
            return controller.leftTrigger();
        }

        @Override
        public Trigger rightTrigger() {
            return controller.rightTrigger();
        }

        @Override
        public Trigger leftBumper() {
            return controller.leftBumper();
        }

        @Override
        public Trigger rightBumper() {
            return controller.rightBumper();
        }

        @Override
        public Trigger povUp() {
            return controller.povUp();
        }

        @Override
        public Trigger povDown() {
            return controller.povDown();
        }

        @Override
        public Trigger faceDown() {
            return controller.a();
        }

        @Override
        public Trigger faceRight() {
            return controller.b();
        }

        @Override
        public Trigger faceLeft() {
            return controller.x();
        }

        @Override
        public Trigger faceUp() {
            return controller.y();
        }

        @Override
        public Trigger menuLeft() {
            return controller.back();
        }
    }

    private static final class Ps5DriverControls implements DriverControls {
        private final CommandPS5Controller controller;

        private Ps5DriverControls(int port) {
            controller = new CommandPS5Controller(port);
        }

        @Override
        public double getLeftX() {
            return controller.getLeftX();
        }

        @Override
        public double getLeftY() {
            return controller.getLeftY();
        }

        @Override
        public double getRightX() {
            return controller.getRightX();
        }

        @Override
        public Trigger leftTrigger() {
            return controller.L2();
        }

        @Override
        public Trigger rightTrigger() {
            return controller.R2();
        }

        @Override
        public Trigger leftBumper() {
            return controller.L1();
        }

        @Override
        public Trigger rightBumper() {
            return controller.R1();
        }

        @Override
        public Trigger povUp() {
            return controller.povUp();
        }

        @Override
        public Trigger povDown() {
            return controller.povDown();
        }

        @Override
        public Trigger faceDown() {
            return controller.cross();
        }

        @Override
        public Trigger faceRight() {
            return controller.circle();
        }

        @Override
        public Trigger faceLeft() {
            return controller.square();
        }

        @Override
        public Trigger faceUp() {
            return controller.triangle();
        }

        @Override
        public Trigger menuLeft() {
            return controller.create();
        }
    }
}
