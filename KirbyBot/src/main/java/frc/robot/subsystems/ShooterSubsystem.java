package frc.robot.subsystems;

import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Ports;


//This is the subsystem for the Shooter, which shoots the balls to target [replace target with more accurate name]
public class ShooterSubsystem extends SubsystemBase {

    private final TalonFX leftMotor, middleMotor, rightMotor;
    private final TalonFX[] motors = new TalonFX[3];
    private final VelocityVoltage velocityRequest = new VelocityVoltage(0).withSlot(0);
    private final VoltageOut voltageRequest = new VoltageOut(0);

    private double dashboardTargetRPM = 0.0;
    private static final AngularVelocity maxVelocityTolerance = RPM.of(100);

    public ShooterSubsystem() {
        leftMotor = new TalonFX(Ports.kShooterLeft, Ports.kRoboRioCANbus);
        middleMotor = new TalonFX(Ports.kShooterMiddle, Ports.kRoboRioCANbus);
        rightMotor = new TalonFX(Ports.kShooterRight, Ports.kRoboRioCANbus);
        motors[0] = leftMotor; 
        motors[1] = middleMotor;
        motors[2] = rightMotor;
        
        //TODO configure motors

        //TODO smartdashboard data adding
    }

    private void configureMotor(TalonFX motor, InvertedValue invertDirection) {
        //use class TalonFXConfiguration and call it config
            //define motor output configs
            //define voltage configs
            //define current limits configs
            //define slot0 configs
        //apply config to motor.getConfigurator()
    }

    public void setRPM(double rpm) {
        for (final TalonFX motor : motors) {
           motor.setControl(velocityRequest.withVelocity(RPM.of(rpm)));
        } 
    }

    public void setPercentOutput(double percentOutput) {
        for (final TalonFX motor : motors) {
           motor.setControl(voltageRequest.withOutput(Volts.of(percentOutput * 12.0)));
        } 
    }

    public void stop() {
        setPercentOutput(0.0);
    }

    public Command spinUpCommand(double rpm){
        return runOnce(() -> setRPM(rpm)).andThen(Commands.waitUntil(this::isVelocityWithinTolerance));
    }

    public Command dashboardSpinUpComand() {
        return defer(() -> spinUpCommand(dashboardTargetRPM));
    }

    public boolean isVelocityWithinTolerance() {
        return false; //DUMMY VALUE
    }


}   



/* 
 * private static final AngularVelocity maxVelocityTolerance = RPM.of(magnitude:100)
 * private final TalonFX leftMotor, middleMotor, rightMotor
 * private final TalonFX[] motors;
 * private final VelocityVoltage velocityRequest = new VelocityVoltage(Velocity:0).withSlot(newSlot:0)
 * private final VoltageOut voltageRequest = new VoltageOut(Output:0)
 * private double dashboardTargetRPM = 0.0
 * 
 * public
 * 
 * 
 * 
 * 
 * 
 * 
 * 
 * constructor ShooterSubsystem() {
 *      set motor variables
 *      put motor variables in motors[]
 *      configure motors (using configureMotor)
 *      put data into smart dashboard
 * }
 * 
 * private void method configureMotor(TalonFX motor, InvertedValue invertDirection) {
 *      use class TalonFXConfiguration and call it config
 *          define motor output configs
 *          define voltage configs
 *          define current limits configs
 *          define slot0 configs
 *      apply config to motor.getConfigurator()
 * }
 * 
 * public void method setRPM(double rpm) {
 *      for (final TalonFX motor go through all motors) {
 *          motor set rpm
 *      } 
 * }
 * 
 * public void method setPercentOutput(double percentOutput) {
 *      for (final TalonFX motor go through all motors) {
 *          motor set percentOutput
 *      } 
 * }
 * 
 * public void method stop() {
 *      set percentOutput 0 
 * }
 * 
 * public method Command dashboardSpinUpComand() {
 *      when told to by the dashboard, return a command to set the rpm
 * }
 * 
 * public method boolean isVelocityWithinTolerance() {
 *      check if velocity is within tolerance
 *      return true if it is, return false if not
 * }
 * 
 * private void method initSendable(SendableBuilder builder, TalonFX motor, String name) {
 *      gives builder info about all of this stuff :thumbs up:
 * }
 * 
 * @Override
 * public void method initSendable(SendableBuilder builder) {
 *      call initSendable (private version) for left, middle, and right motor
 *      adds more info to builder
 * }
*/