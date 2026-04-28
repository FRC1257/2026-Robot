package frc.robot.util.misc;

import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.nio.file.StandardOpenOption;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Properties;
import java.util.function.Consumer;
import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import frc.robot.Constants;

/**
 * A tunable number that persists its value to disk on the RoboRIO, so tuned values survive
 * restarts without redeployment. Combines NetworkTables live tuning with file-based persistence.
 *
 * <p>Lifecycle:
 * <ol>
 *   <li>On first construction, loads all values from the persist file (if it exists).
 *   <li>Each number uses the stored value as its initial value, falling back to the compiled default
 *       when no stored value is found.
 *   <li>When a value changes via NetworkTables (dashboard), it is written back to the file
 *       immediately so the next startup picks it up.
 *   <li>Call {@link #reloadAll()} from a DigitalInput trigger to re-read the file mid-run and push
 *       updated values to the dashboard without restarting.
 * </ol>
 *
 * <p>The persist file is a standard Java Properties file (plain text, {@code key=value}).
 * It can be edited with any text editor via {@code ssh} or {@code sftp}.
 */
@SuppressWarnings("unused")
public class PersistingLoggedTunableNumber implements DoubleSupplier {

    private static final Path PERSIST_FILE =
            (Constants.currentMode == Constants.Mode.SIM || Constants.currentMode == Constants.Mode.REPLAY)
                    ? Paths.get("sim_tuning.properties")
                    : Paths.get("/home/lvuser/tuning.properties");

    private static final String TABLE_KEY = "/Tuning";

    private static final Properties sharedProperties = new Properties();
    private static boolean loaded = false;
    private static boolean pruned = false;

    private static final List<PersistingLoggedTunableNumber> instances = new ArrayList<>();

    private final String key;
    private final String fullNtKey;
    private boolean hasDefault = false;
    private double defaultValue;
    private LoggedNetworkNumber dashboardNumber;
    private final Map<Integer, Double> lastHasChangedValues = new HashMap<>();
    private double lastSavedValue = Double.NaN;

    /**
     * Creates a new PersistingLoggedTunableNumber without a default value.
     * Call {@link #initDefault} before using.
     *
     * @param dashboardKey key displayed on the dashboard and used as the property file key
     */
    public PersistingLoggedTunableNumber(String dashboardKey) {
        this.key = dashboardKey;
        this.fullNtKey = TABLE_KEY + "/" + dashboardKey;
        ensureLoaded();
        if (Constants.tuningMode && !Constants.disableHAL) {
            String stored = sharedProperties.getProperty(dashboardKey);
            double initial = 0.0;
            if (stored != null) {
                try { initial = Double.parseDouble(stored); } catch (NumberFormatException ignored) {}
            }
            dashboardNumber = new LoggedNetworkNumber(fullNtKey, initial);
            lastSavedValue = initial;
        }
        synchronized (instances) {
            instances.add(this);
        }
    }

    /**
     * Creates a new PersistingLoggedTunableNumber with a default value.
     *
     * @param dashboardKey key displayed on the dashboard and used as the property file key
     * @param defaultValue compiled-in default; overridden by any stored value in the persist file
     */
    public PersistingLoggedTunableNumber(String dashboardKey, double defaultValue) {
        this(dashboardKey);
        initDefault(defaultValue);
    }

    /**
     * Sets the default value. May only be called once; subsequent calls are ignored.
     * If the persist file already has a value for this key, that stored value is used instead.
     *
     * @param defaultValue compiled-in default
     */
    public void initDefault(double defaultValue) {
        if (hasDefault) return;
        hasDefault = true;

        String stored = sharedProperties.getProperty(key);
        double initial = defaultValue;
        if (stored != null) {
            try {
                initial = Double.parseDouble(stored);
            } catch (NumberFormatException ignored) {
                // Corrupt entry — fall back to compiled default and overwrite below.
                stored = null;
            }
        }

        this.defaultValue = initial;
        this.lastSavedValue = initial; // Avoid an immediate write on the first get() call.

        if (stored == null) {
            // New key — write the default to the file so it's visible for manual editing.
            persistValue(key, initial);
        }

        if (Constants.tuningMode && !Constants.disableHAL) {
            if (dashboardNumber == null) {
                dashboardNumber = new LoggedNetworkNumber(fullNtKey, initial);
            } else {
                dashboardNumber.set(initial);
            }
        }
    }

    /**
     * Returns the current value. In tuning mode the value comes from NetworkTables; otherwise the
     * (possibly file-loaded) default is returned. Any change detected here is immediately written
     * to the persist file.
     *
     * @return current tunable value
     */
    public double get() {
        if (!hasDefault) return 0.0;

        double value = (Constants.tuningMode && !Constants.disableHAL)
                ? dashboardNumber.get()
                : defaultValue;

        if (Double.compare(value, lastSavedValue) != 0) {
            lastSavedValue = value;
            persistValue(key, value);
        }

        return value;
    }

    /**
     * Checks whether the value has changed since the last call with the same {@code id}.
     *
     * @param id unique identifier for the caller; use {@code hashCode()} as a simple choice
     * @return {@code true} if the value changed since the last call with this {@code id}
     */
    public boolean hasChanged(int id) {
        double currentValue = get();
        Double lastValue = lastHasChangedValues.get(id);
        if (lastValue == null || currentValue != lastValue) {
            lastHasChangedValues.put(id, currentValue);
            return true;
        }
        return false;
    }

    /**
     * Runs {@code action} with the current values of all {@code numbers} if any of them changed.
     *
     * @param id            unique identifier for the caller
     * @param action        receives the values of all numbers in the order they were passed
     * @param numbers       numbers to watch
     */
    public static void ifChanged(
            int id, Consumer<double[]> action, PersistingLoggedTunableNumber... numbers) {
        if (Arrays.stream(numbers).anyMatch(n -> n.hasChanged(id))) {
            action.accept(Arrays.stream(numbers).mapToDouble(PersistingLoggedTunableNumber::get).toArray());
        }
    }

    /** Runs {@code action} if any of {@code numbers} changed. */
    public static void ifChanged(int id, Runnable action, PersistingLoggedTunableNumber... numbers) {
        ifChanged(id, values -> action.run(), numbers);
    }

    /**
     * Re-reads the persist file and pushes any updated values to the dashboard.
     *
     * <p>Intended to be called from a {@code DigitalInput}-triggered command so the robot can
     * reconfigure mid-run without a restart or redeployment.
     */
    public static synchronized void reloadAll() {
        sharedProperties.clear();
        loadFromFile();

        synchronized (instances) {
            for (PersistingLoggedTunableNumber n : instances) {
                if (!n.hasDefault) continue;
                String stored = sharedProperties.getProperty(n.key);
                if (stored == null) continue;
                try {
                    double newValue = Double.parseDouble(stored);
                    if (n.dashboardNumber != null) {
                        n.dashboardNumber.set(newValue);
                    } else {
                        n.defaultValue = newValue;
                    }
                    n.lastSavedValue = newValue;
                    // Reset hasChanged trackers so callers notice the reload.
                    n.lastHasChangedValues.clear();
                } catch (NumberFormatException ignored) {}
            }
        }
    }

    /**
     * Polls every registered instance so that dashboard changes are written to disk even when no
     * command is actively calling {@link #get()}. Call this once per loop from
     * {@code Robot.robotPeriodic()}.
     */
    public static void periodic() {
        if (!pruned) {
            pruned = true;
            pruneStaleKeys();
        }
        synchronized (instances) {
            for (PersistingLoggedTunableNumber n : instances) {
                if (n.hasDefault) {
                    n.get();
                } else if (n.dashboardNumber != null) {
                    double value = n.dashboardNumber.get();
                    if (Double.compare(value, n.lastSavedValue) != 0) {
                        n.lastSavedValue = value;
                        persistValue(n.key, value);
                    }
                }
            }
        }
    }

    @Override
    public double getAsDouble() {
        return get();
    }

    // -------------------------------------------------------------------------
    // File I/O helpers
    // -------------------------------------------------------------------------

    private static synchronized void pruneStaleKeys() {
        boolean changed = false;
        synchronized (instances) {
            for (String fileKey : sharedProperties.stringPropertyNames()) {
                boolean live = instances.stream().anyMatch(n -> n.key.equals(fileKey));
                if (!live) {
                    sharedProperties.remove(fileKey);
                    changed = true;
                }
            }
        }
        if (!changed) return;
        try (OutputStream out = Files.newOutputStream(
                PERSIST_FILE, StandardOpenOption.CREATE, StandardOpenOption.TRUNCATE_EXISTING)) {
            sharedProperties.store(out, "Robot tuning — edit via ssh/sftp, reload with reloadAll(), no redeployment needed");
        } catch (IOException e) {
            System.err.println("[PersistingLoggedTunableNumber] Failed to prune "
                    + PERSIST_FILE + ": " + e.getMessage());
        }
    }

    private static synchronized void ensureLoaded() {
        if (loaded) return;
        loaded = true;
        loadFromFile();
    }

    private static void loadFromFile() {
        if (!Files.exists(PERSIST_FILE)) return;
        try (InputStream in = Files.newInputStream(PERSIST_FILE)) {
            sharedProperties.load(in);
        } catch (IOException e) {
            System.err.println("[PersistingLoggedTunableNumber] Failed to load "
                    + PERSIST_FILE + ": " + e.getMessage());
        }
    }

    private static synchronized void persistValue(String key, double value) {
        sharedProperties.setProperty(key, Double.toString(value));
        try (OutputStream out = Files.newOutputStream(
                PERSIST_FILE, StandardOpenOption.CREATE, StandardOpenOption.TRUNCATE_EXISTING)) {
            sharedProperties.store(out, "Robot tuning — edit via ssh/sftp, reload with reloadAll(), no redeployment needed");
        } catch (IOException e) {
            System.err.println("[PersistingLoggedTunableNumber] Failed to save "
                    + PERSIST_FILE + ": " + e.getMessage());
        }
    }
}
