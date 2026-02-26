package frc.robot.util.misc;

import java.util.HashMap;
import java.util.Map;
import java.util.function.Supplier;

import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Unit;
import frc.robot.Constants;

public class LoggedTunableMeasure<U extends Unit> implements Supplier<Measure<U>> {
   
    private static final String TABLE_KEY = "/Tuning";

    private final String key;
    private final U displayUnit;

    private boolean hasDefault = false;
    private Measure<U> defaultValue;
    private LoggedNetworkNumber dashboardNumber;

    private final Map<Integer, Double> lastHasChangedValues = new HashMap<>();

    /**
     * Create a new LoggedTunableMeasure
     * @param dashboardKey Key on dashboard
     * @param displayUnit Unit to display the measure in on the dashboard
     */

    public LoggedTunableMeasure(String dashboardKey, U displayUnit) {
        this.key = TABLE_KEY + "/" + dashboardKey;
        this.displayUnit = displayUnit;
    }

    /**
     * Create a new LoggedTunableMeasure with the default value
     * @param dashboardKey Key on dashboard
     * @param defaultValue Default value, also used to determine the display unit
     */

    public LoggedTunableMeasure(String dashboardKey, Measure<U> defaultValue) {
        this(dashboardKey, defaultValue.unit());
        initDefault(defaultValue);
    }

    /**
     * Set the default value of the measure.
     * @param defaultValue The default value, also used to determine the display unit. The default value can only be set once.
     */

    public void initDefault(Measure<U> defaultValue) {
        if (!hasDefault) {
            hasDefault = true;
            this.defaultValue = defaultValue;
            if (Constants.tuningMode && !Constants.disableHAL) {
                dashboardNumber = new LoggedNetworkNumber(key, defaultValue.in(displayUnit));
            }
        }
    }

    /**
     * Get the value of the measure. If in tuning mode and the value is in the dashboard, get it from the dashboard, otherwise return the default value. If no default value has been set, return zero in the display unit.
      * @return The value of the measure
      * @throws ClassCastException if no default value has been set and the display unit cannot be cast to a measure of type U
     */

    @SuppressWarnings({ "unchecked", "unused" })
    @Override
    public Measure<U> get() throws ClassCastException {
        if(!hasDefault) {
            return (Measure<U>) displayUnit.zero();
        }
        double value = (Constants.tuningMode && !Constants.disableHAL) ? dashboardNumber.get() : defaultValue.in(displayUnit);
        return (Measure<U>) displayUnit.of(value);
    }

    /**
     * Get the value of the measure in the specified unit as a double.
     * @return the value of the measure in the specified unit as a double
     */

    public double in(U unit) {
        return get().in(unit);
    }

    /**
     * Get the display unit of the measure.
     * @return The display unit of the measure
     */

    public U unit() {
        return displayUnit;
    }

    /**
     * Check if the value of the measure has changed since the last time this method was called with the same id.
     * @param id An identifier for the value being checked 
     * @return true if the value has changed, false otherwise
     */
    
    public boolean hasChanged(int id) {

        double currentValue = get().in(displayUnit);
        Double lastValue = lastHasChangedValues.get(id);

        if(lastValue == null || currentValue != lastValue) {
            lastHasChangedValues.put(id, currentValue);
            return true;
        }
        return false;
    }
    
}
