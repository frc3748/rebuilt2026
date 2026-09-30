package frc.robot.util;

import java.util.Map.Entry;
import java.util.Optional;
import java.util.concurrent.ConcurrentNavigableMap;
import java.util.concurrent.ConcurrentSkipListMap;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.interpolation.Interpolatable;
import edu.wpi.first.math.interpolation.Interpolator;

public final class ConcurrentTimeInterpolatableBuffer<T> {
    private final double m_historySize;
    private final Interpolator<T> m_interpolatingFunc;
    private final ConcurrentNavigableMap<Double, T> m_pastSnapshots = new ConcurrentSkipListMap<>();

    private ConcurrentTimeInterpolatableBuffer(Interpolator<T> interpolateFunction, double historySizeSeconds) {
        this.m_historySize = historySizeSeconds;
        this.m_interpolatingFunc = interpolateFunction;
    }

    public static <T> ConcurrentTimeInterpolatableBuffer<T> createBuffer(
            Interpolator<T> interpolateFunction, double historySizeSeconds) {
        return new ConcurrentTimeInterpolatableBuffer<>(interpolateFunction, historySizeSeconds);
    }

    public static <T extends Interpolatable<T>> ConcurrentTimeInterpolatableBuffer<T> createBuffer(
            double historySizeSeconds) {
        return new ConcurrentTimeInterpolatableBuffer<>(Interpolatable::interpolate, historySizeSeconds);
    }

    public static ConcurrentTimeInterpolatableBuffer<Double> createDoubleBuffer(double historySizeSeconds) {
        return new ConcurrentTimeInterpolatableBuffer<>(MathUtil::interpolate, historySizeSeconds);
    }

    public void addSample(double timeSeconds, T sample) {
        m_pastSnapshots.put(timeSeconds, sample);
        cleanUp(timeSeconds);
    }

    private void cleanUp(double time) {
        m_pastSnapshots.headMap(time - m_historySize, false).clear();
    }

    public void clear() {
        m_pastSnapshots.clear();
    }

    public Optional<T> getSample(double timeSeconds) {
        if (m_pastSnapshots.isEmpty()) {
            return Optional.empty();
        }

        var nowEntry = m_pastSnapshots.get(timeSeconds);
        if (nowEntry != null) {
            return Optional.of(nowEntry);
        }

        var bottomBound = m_pastSnapshots.floorEntry(timeSeconds);
        var topBound = m_pastSnapshots.ceilingEntry(timeSeconds);

        if (topBound == null && bottomBound == null) {
            return Optional.empty();
        } else if (topBound == null) {
            return Optional.of(bottomBound.getValue());
        } else if (bottomBound == null) {
            return Optional.of(topBound.getValue());
        } else {
            return Optional.of(
                    m_interpolatingFunc.interpolate(
                            bottomBound.getValue(),
                            topBound.getValue(),
                            (timeSeconds - bottomBound.getKey()) / (topBound.getKey() - bottomBound.getKey())));
        }
    }

    public Entry<Double, T> getLatest() {
        return m_pastSnapshots.lastEntry();
    }

    public ConcurrentNavigableMap<Double, T> getInternalBuffer() {
        return m_pastSnapshots;
    }
}