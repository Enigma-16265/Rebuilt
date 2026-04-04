package frc.util;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.numbers.N2;

import edu.wpi.first.math.filter.SlewRateLimiter;


public class DriveInputSmoother {
    private static final double kJoystickDeadband = 0.15;
    private static final double kCurveExponent = 1.5;

    private static final double kTranslationSlewRate = 2.0;

    private final SlewRateLimiter forwardLimiter = new SlewRateLimiter(kTranslationSlewRate);
    private final SlewRateLimiter leftLimiter = new SlewRateLimiter(kTranslationSlewRate);

    private final DoubleSupplier forwardInput;
    private final DoubleSupplier leftInput;
    private final DoubleSupplier rotationInput;

    public DriveInputSmoother(DoubleSupplier forwardInput, DoubleSupplier leftInput, DoubleSupplier rotationInput) {
        this.forwardInput = forwardInput;
        this.leftInput = leftInput;
        this.rotationInput = rotationInput;
    }

    public DriveInputSmoother(DoubleSupplier forwardInput, DoubleSupplier leftInput) {
        this(forwardInput, leftInput, () -> 0);
    }

    public void reset() {
        forwardLimiter.reset(0);
        leftLimiter.reset(0);
    }

    public ManualDriveInput getSmoothedInput() { 
        final Vector<N2> rawTranslationInput = VecBuilder.fill(forwardInput.getAsDouble(), leftInput.getAsDouble());
        final Vector<N2> deadbandedTranslationInput = MathUtil.applyDeadband(rawTranslationInput, kJoystickDeadband);
        final Vector<N2> curvedTranslationInput = MathUtil.copyDirectionPow(deadbandedTranslationInput, kCurveExponent);

        final double smoothedForward = forwardLimiter.calculate(curvedTranslationInput.get(0));
        final double smoothedLeft = leftLimiter.calculate(curvedTranslationInput.get(1));

        final double rawRotationInput = rotationInput.getAsDouble();
        final double deadbandedRotationInput = MathUtil.applyDeadband(rawRotationInput, kJoystickDeadband);
        final double curvedRotationInput = MathUtil.copyDirectionPow(deadbandedRotationInput, kCurveExponent);

        return new ManualDriveInput(
            smoothedForward,
            smoothedLeft,
            curvedRotationInput
        );
    }

}
