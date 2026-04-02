package frc.lib.tuning;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Consumer;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Preferences;

/**
 * A NetworkTables-backed tunable number with optional listeners and preference
 * persistence. When tuning is active, values are mirrored to Preferences and
 * updates are polled at a throttled interval; listeners fire only when the
 * value changes.
 */
public class TunableNumber {
    private static final String NT_TABLE = "Tuning";
    
    private static final long UPDATE_INTERVAL_MS = 1000L; // Check for updates every 1000ms
    private static long lastPollTime = 0L;

    private static final List<TunableNumber> allInstances = new ArrayList<>();

    private final String key;
    private final double defaultValue;
    private final List<Consumer<Double>> listeners = new ArrayList<>(2);

    // lastValue: last value delivered to listeners via refreshIfNeeded()
    // lastWrittenValue: last value written into Preferences from NetworkTables
    private double lastValue;
    private double lastWrittenValue;
    private NetworkTableEntry ntEntry;

    /**
     * Creates a tunable number. If tuning is active, seeds NetworkTables from
     * Preferences (initializing the preference key if missing) and starts tracking
     * this instance for periodic refresh.
     *
     * @param key           preference/NT key (e.g. "Drive/kP" -> /Tuning/Drive/kP)
     * @param defaultValue  fallback when tuning inactive or key absent
     */
    public TunableNumber(String key, double defaultValue) {
        this.key = key;
        this.defaultValue = defaultValue;
 
        allInstances.add(this);
 
        if (Tuning.isActive()) {
            if (!Preferences.containsKey(key)) {
                Preferences.initDouble(key, defaultValue);
            }
 
            // Build NT hierarchy from key segments
            // e.g. "Drive/kP" → /Tuning/Drive/kP
            String[] parts = key.split("/");
            NetworkTable table = NetworkTableInstance.getDefault().getTable(NT_TABLE);
            for (int i = 0; i < parts.length - 1; i++) {
                table = table.getSubTable(parts[i]);
            }
            ntEntry = table.getEntry(parts[parts.length - 1]);
 
            // Seed NT from Preferences so last tuned value is immediately visible
            double initialValue = Preferences.getDouble(key, defaultValue);
            ntEntry.setDouble(initialValue);
            lastWrittenValue = initialValue;
            lastValue = initialValue;
        } else {
            lastWrittenValue = defaultValue;
            lastValue = defaultValue;
        }
    }

    /**
     * Returns the current value from NetworkTables (or default if tuning is
     * inactive) and writes it to Preferences if it changed since the last write.
     *
     * @return latest value
     */
    public double getAndUpdatePref() {
        if (!Tuning.isActive()) {
            return defaultValue;
        }
 
        double ntValue = ntEntry.getDouble(defaultValue);
 
        // Only write to Preferences when the value actually changes
        if (ntValue != lastWrittenValue) {
            Preferences.setDouble(key, ntValue);
            lastWrittenValue = ntValue;
        }
 
        return ntValue;
    }

    /**
     * Registers a listener invoked when the value changes (detected via
     * {@link #refreshIfNeeded()}).
     *
     * @param consumer callback receiving the updated value
     */
    public void onChange(Consumer<Double> consumer) {
        listeners.add(consumer);
    }

    /** Clears all registered listeners. */
    void clearListeners() {
        listeners.clear();
    }

    /**
     * Polls all instances at a throttled interval, writing changed values to
     * Preferences and notifying listeners when a value changes. No-ops when
     * tuning is inactive.
     */
    public static void refreshIfNeeded() {
        if(!Tuning.isActive()) {
            return;
        }

        final long now = System.currentTimeMillis();
        if(now - lastPollTime < UPDATE_INTERVAL_MS) {
            return;
        }
        for (TunableNumber t : allInstances) {
            double current = t.getAndUpdatePref();

            // Notify listeners only when the value changes since the last notification.
            if (current != t.lastValue) {
                t.lastValue = current;
                for (Consumer<Double> listener : t.listeners) {
                    listener.accept(current);
                }
            }
        }
        lastPollTime = now;
    }

    /**
     * Emits a warning for any stored preference that deviates from its default,
     * prompting operators to promote or clear stale tuned values.
     */
    public static void checkAllForStalePreferences() {
        for (TunableNumber t : allInstances) {
            if (Preferences.containsKey(t.key)) {
                double stored = Preferences.getDouble(t.key, t.defaultValue);
                if (stored != t.defaultValue) {
                    DriverStation.reportWarning(
                        "[TunableNumber] Stale tuned value for \"" + t.key + "\": " +
                        "stored=" + stored + ", default=" + t.defaultValue +
                        ". Either promote this value or clear Preferences.", false
                    );
                }
            }
        }
    }
}