package frc.robot.commands;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Floor;
import frc.robot.subsystems.Hanger;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;

public final class SimpleAuto {
    private static final LinearVelocity kForwardSpeed = MetersPerSecond.of(-0.5);
    private static final Time kDriveTime = Seconds.of(1);
    private static final Time kShootTimeout = Seconds.of(5.0);

    private final Swerve swerve;
    private final SubsystemCommands subsystemCommands;

    private final SwerveRequest.RobotCentric driveForwardRequest = new SwerveRequest.RobotCentric()
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
        .withSteerRequestType(SteerRequestType.MotionMagicExpo);
    private final SwerveRequest.SwerveDriveBrake brakeRequest = new SwerveRequest.SwerveDriveBrake();

    private final Intake intake;

    public SimpleAuto(
        Swerve swerve,
        Intake intake,
        Floor floor,
        Feeder feeder,
        Shooter shooter,
        Hood hood,
        Hanger hanger
    ) {
        this.swerve = swerve;
        this.subsystemCommands = new SubsystemCommands(swerve, intake, floor, feeder, shooter, hood, hanger);
        this.intake = intake;
    }

    public Command command() {
        return Commands.sequence(
            Commands.runOnce(() -> swerve.resetPose(swerve.getState().Pose), swerve),
            driveForward(),
            intake.intakeCommand(),
            subsystemCommands.aimAndShoot().withTimeout(kShootTimeout.in(Seconds))
        ).withName("SimpleAuto");
    }

    private Command driveForward() {
        return swerve.applyRequest(() ->
            driveForwardRequest.withVelocityX(kForwardSpeed)
        )
        .withTimeout(kDriveTime.in(Seconds))
        .andThen(swerve.applyRequest(() -> brakeRequest).withTimeout(0.1));
    }
}
