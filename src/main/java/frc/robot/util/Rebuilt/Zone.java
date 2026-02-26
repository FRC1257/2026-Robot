package frc.robot.util.Rebuilt;

import java.util.function.BinaryOperator;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public interface Zone {

    /**
     * Returns a Trigger that is active when the given translation is within the zone.
     * @param translation the translation to check 
     * @return a Trigger that is active when the given translation is within the zone
     */

    public Trigger contains(Supplier<Translation2d> translation);

    /**
     * Returns a new Zone that is the composition of this zone and another zone using the given operator.
     * @param other the other zone to compose with
     * @param operator the operator to use for composition (e.g. Trigger::or for union, Trigger::and for intersection)
     * @return a new Zone that is the composition of this zone and the other zone using the given operator
     */

    public default Zone compose(Zone other, BinaryOperator<Trigger> operator) {
        return t -> operator.apply(this.contains(t), other.contains(t));
    }

    /**
     * Returns a new Zone that is the union of this zone and another zone.
     * @param other the other zone to union with
     * @return a new Zone that is the union of this zone and the other zone
     */

    public default Zone union(Zone other) {
        return compose(other, Trigger::or);
    }

    /**
     * Returns a new Zone that is the intersection of this zone and another zone.
     * @param other the other zone to intersect with
     * @return a new Zone that is the intersection of this zone and the other zone
     */

    public default Zone intersection(Zone other) {
        return compose(other, Trigger::and);
    }

    /**
     * Returns a new Zone that is the difference of this zone and another zone (i.e. the area that is in this zone but not in the other zone).
     * @param other the other zone to subtract from this zone
     * @return a new Zone that is the difference of this zone and the other zone
     */
    
    public default Zone difference(Zone other) {
        return compose(other, (a, b) -> a.and(b.negate()));
    }
}
