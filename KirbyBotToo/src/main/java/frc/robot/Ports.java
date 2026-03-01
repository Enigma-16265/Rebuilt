package frc.robot;

import com.ctre.phoenix6.CANBus;

public final class Ports {
    // CAN Buses
    public static final CANBus kRoboRioCANBus = new CANBus("rio");
    public static final CANBus kCANivoreCANBus = new CANBus("main");

    // Talon FX IDs
    public static final int kIntakePivot = 52;
    public static final int kIntakeRollers = 51;
    public static final int kFloor = 54;
    public static final int kFeeder = 55;
    public static final int kShooterLeft = 56;
    public static final int kShooterMiddle = 57;
    public static final int kShooterRight = 58;
    public static final int kHanger = 53;

    // PWM Ports
    public static final int kHoodLeftServo  = 8;
    public static final int kHoodRightServo = 9;
}
