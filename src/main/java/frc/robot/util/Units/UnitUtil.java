package frc.robot.util.Units;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.interpolation.Interpolator;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Unit;

public class UnitUtil { 

    /**
     * Preserve the concrete Measure subtype when clamping. Useful when callers expect
     * to receive the same runtime type as the inputs (for example, {@code Angle}).
     * This performs an unchecked cast back to {@code M} after using the generic
     * Measure-based min/max operations
     *
     * @param value the value to clamp
     * @param low the minimum allowed value
     * @param high the maximum allowed value
     * @param <U> the unit type
     * @param <M> the concrete Measure subtype
     * @return the clamped value as the same concrete subtype M
     */

    @SuppressWarnings("unchecked")
    public static <U extends Unit, M extends Measure<U>> M clamp(M value, M low, M high) {
        return (M) Measure.max(low, Measure.min(value, high));
    }

    /**
     * Linearly interpolates between two {@link Measure} values of the same unit type.
     * The interpolation parameter {@code t} is clamped to [0, 1], so values outside
     * that range will not extrapolate beyond {@code a} or {@code b}.
     *
     * @param <U> the unit type (e.g. {@link edu.wpi.first.units.AngleUnit})
     * @param <M> the concrete {@link Measure} subtype (e.g. {@link edu.wpi.first.units.measure.Angle})
     * @param a the start value, returned when {@code t = 0}
     * @param b the end value, returned when {@code t = 1}
     * @param t the interpolation parameter, clamped to [0, 1]
     * @return the interpolated value as the same concrete subtype {@code M}
     */

    @SuppressWarnings("unchecked")
    public static <U extends Unit, M extends Measure<U>> M interpolate(M a, M b, double t) {
        return (M) a.plus(b.minus(a).times(MathUtil.clamp(t, 0.0, 1.0)));
    }
    
    /**
     * Returns a reusable {@link Interpolator} for {@link Measure} values of the given unit type.
     * The returned interpolator delegates to {@link #interpolate(Measure, Measure, double)},
     * clamping {@code t} to [0, 1] and preserving the concrete {@link Measure} subtype.

     * @param <U> the unit type (e.g. {@link edu.wpi.first.units.AngleUnit})
     * @param <M> the concrete {@link Measure} subtype (e.g. {@link edu.wpi.first.units.measure.Angle})
     * @param unit the unit to create the interpolator for — not used in interpolation directly, but conveys intent at the call site (e.g. {@code interpolatorFor(Radians)})
     * @return a {@link Interpolator} that linearly interpolates between two {@code M} values
     */

    public static <U extends Unit, M extends Measure<U>> Interpolator<Measure<U>> lerpOf(U unit) {
        return UnitUtil::interpolate;
    }

    /**
     * Computes the inverse linear interpolation of {@code query} between {@code start} and {@code end},
     * returning the parameter {@code t} in [0, 1] such that
     * {@link #interpolate(Measure, Measure, double) interpolate(start, end, t)} would produce {@code query}.
     *
     * @param <U> the unit type (e.g. {@link edu.wpi.first.units.AngleUnit})
     * @param <M> the concrete {@link Measure} subtype (e.g. {@link edu.wpi.first.units.measure.Angle})
     * @param start the start of the range, corresponding to {@code t = 0}
     * @param end the end of the range, corresponding to {@code t = 1}
     * @param query the value to find the interpolation parameter for
     * @return the interpolation parameter {@code t} in [0, 1] representing {@code query}'s position between {@code start} and {@code end}
     */

    public static <U extends Unit, M extends Measure<U>> double inverseInterpolate(M start, M end, M query) {
        double range = end.minus(start).baseUnitMagnitude();
        if(Math.abs(range) < 1e-12) return 0.0;
        return MathUtil.clamp(query.minus(start).div(end.minus(start)).baseUnitMagnitude(), 0.0, 1.0);
    }

    /**
     * Returns a reusable {@link InverseInterpolator} for {@link Measure} values of the given unit type.
     * The returned inverse interpolator delegates to {@link #inverseInterpolate(Measure, Measure, Measure)},
     * clamping results to [0, 1] and handling zero-range inputs safely.
     *
     * @param <U>  the unit type (e.g. {@link edu.wpi.first.units.AngleUnit})
     * @param <M>  the concrete {@link Measure} subtype (e.g. {@link edu.wpi.first.units.measure.Angle})
     * @param unit the unit to create the inverse interpolator for — not used in computation directly, but conveys intent at the call site (e.g. {@code inverseLerpOf(Radians)})
     * @return an {@link InverseInterpolator} that computes the inverse linear interpolation parameter for two {@code Measure<U>} bounds and a query value
     */
    
    public static <U extends Unit, M extends Measure<U>> InverseInterpolator<Measure<U>> inverseLerpOf(U unit) {
        return UnitUtil::inverseInterpolate;
    }

}
