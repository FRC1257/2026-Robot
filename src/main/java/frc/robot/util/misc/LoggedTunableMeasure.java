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

    public LoggedTunableMeasure(String dashboardKey, U displayUnit) {
        this.key = TABLE_KEY + "/" + dashboardKey;
        this.displayUnit = displayUnit;
    }

    public LoggedTunableMeasure(String dashboardKey, Measure<U> defaultValue) {
        this(dashboardKey, defaultValue.unit());
        initDefault(defaultValue);
    }

    public void initDefault(Measure<U> defaultValue) {
        if (!hasDefault) {
            hasDefault = true;
            this.defaultValue = defaultValue;
            if (Constants.tuningMode && !Constants.disableHAL) {
                dashboardNumber = new LoggedNetworkNumber(key, defaultValue.in(displayUnit));
            }
        }
    }

    @SuppressWarnings("unchecked")
    @Override
    public Measure<U> get() {
        if(!hasDefault) {
            return (Measure<U>) displayUnit.zero();
        }
        double value = (Constants.tuningMode && !Constants.disableHAL) ? dashboardNumber.get() : defaultValue.in(displayUnit);
        return (Measure<U>) displayUnit.of(value);
    }

    public double in(U unit) {
        return get().in(unit);
    }

    public U unit() {
        return displayUnit;
    }

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
