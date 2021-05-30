package org.ros.time;

import com.google.common.annotations.VisibleForTesting;
import com.google.common.base.Preconditions;
import com.google.common.collect.Lists;
import java.util.Collections;
import java.util.List;
import java.util.Queue;
import java.util.concurrent.Callable;
import org.apache.commons.logging.Log;
import org.apache.commons.logging.LogFactory;
import org.bytedeco.javacpp.opencv_stitching;
import org.ros.exception.RosRuntimeException;

public class RemoteUptimeClock {
    private static final boolean DEBUG = false;
    private static final Log log = LogFactory.getLog(RemoteUptimeClock.class);
    private final Callable<Double> callable;
    private double drift;
    private final double driftSensitivity;
    private double errorReductionCoefficient;
    private final double errorReductionCoefficientSensitivity;
    private final LatencyOutlierFilter latencyOutlierFilter;
    private double localUptime;
    private final LocalUptimeProvider localUptimeProvider;
    private double measuredRemoteUptime;
    private double predictedRemoteUptime;

    @VisibleForTesting
    interface LocalUptimeProvider {
        double getSeconds();
    }

    private final class UptimeCalculationResult {
        final double latency;
        final double newLocalUptime;
        final double newRemoteUptime;

        public UptimeCalculationResult(double newLocalUptime2, double newRemoteUptime2, double latency2) {
            this.newLocalUptime = newLocalUptime2;
            this.newRemoteUptime = newRemoteUptime2;
            this.latency = latency2;
        }
    }

    private final class LatencyOutlierFilter {
        private final Queue<Double> latencies;
        private final int sampleSize;
        private final double threshold;

        public LatencyOutlierFilter(int sampleSize2, double threshold2) {
            boolean z = false;
            Preconditions.checkArgument(sampleSize2 > 0);
            Preconditions.checkArgument(threshold2 > 1.0d ? true : z);
            this.threshold = threshold2;
            this.sampleSize = sampleSize2;
            this.latencies = Lists.newLinkedList();
        }

        public boolean add(double latency) {
            this.latencies.add(Double.valueOf(latency));
            if (this.latencies.size() <= this.sampleSize) {
                return false;
            }
            this.latencies.remove();
            if (latency < this.threshold * getMedian()) {
                return false;
            }
            return true;
        }

        public double getMedian() {
            List<Double> ordered = Lists.newArrayList(this.latencies);
            Collections.sort(ordered);
            return ordered.get(this.latencies.size() / 2).doubleValue();
        }
    }

    public static RemoteUptimeClock newDefault(final TimeProvider timeProvider, Callable<Double> callable2, double driftSensitivity2, double errorReductionCoefficientSensitivity2, int latencyOutlierFilterSampleSize, double latencyOutlierFilterThreshold) {
        TimeProvider timeProvider2 = timeProvider;
        return new RemoteUptimeClock(new LocalUptimeProvider() {
            public double getSeconds() {
                return timeProvider.getCurrentTime().toSeconds();
            }
        }, callable2, driftSensitivity2, errorReductionCoefficientSensitivity2, latencyOutlierFilterSampleSize, latencyOutlierFilterThreshold);
    }

    @VisibleForTesting
    RemoteUptimeClock(LocalUptimeProvider localUptimeProvider2, Callable<Double> callable2, double driftSensitivity2, double errorReductionCoefficientSensitivity2, int latencyOutlierFilterSampleSize, double latencyOutlierFilterThreshold) {
        double d = driftSensitivity2;
        double d2 = errorReductionCoefficientSensitivity2;
        boolean z = false;
        Preconditions.checkArgument(d >= opencv_stitching.Stitcher.ORIG_RESOL && d <= 1.0d);
        if (d2 >= opencv_stitching.Stitcher.ORIG_RESOL && d2 <= 1.0d) {
            z = true;
        }
        Preconditions.checkArgument(z);
        this.localUptimeProvider = localUptimeProvider2;
        this.callable = callable2;
        this.driftSensitivity = d;
        this.errorReductionCoefficientSensitivity = d2;
        this.latencyOutlierFilter = new LatencyOutlierFilter(latencyOutlierFilterSampleSize, latencyOutlierFilterThreshold);
        this.errorReductionCoefficient = opencv_stitching.Stitcher.ORIG_RESOL;
    }

    public void calibrate(int sampleSize, double samplingDelayMillis) {
        int i;
        int i2 = sampleSize;
        log.info("Starting calibration...");
        double driftSum = opencv_stitching.Stitcher.ORIG_RESOL;
        double localUptimeSum = 0.0d;
        double remoteUptimeSum = 0.0d;
        int i3 = 0;
        while (i3 < i2) {
            UptimeCalculationResult result = calculateNewUptime(this.callable);
            this.latencyOutlierFilter.add(result.latency);
            if (i3 > 0) {
                i = i3;
                driftSum += calculateDrift(result.newLocalUptime - this.localUptime, result.newRemoteUptime - this.measuredRemoteUptime);
            } else {
                i = i3;
            }
            this.measuredRemoteUptime = result.newRemoteUptime;
            this.localUptime = result.newLocalUptime;
            remoteUptimeSum += this.measuredRemoteUptime;
            localUptimeSum += this.localUptime;
            try {
                Thread.sleep((long) samplingDelayMillis);
                i3 = i + 1;
                i2 = sampleSize;
            } catch (InterruptedException e) {
                throw new RosRuntimeException((Throwable) e);
            }
        }
        double d = samplingDelayMillis;
        int i4 = sampleSize;
        double d2 = (double) (i4 - 1);
        Double.isNaN(d2);
        this.drift = driftSum / d2;
        double d3 = (double) i4;
        Double.isNaN(d3);
        double offset = ((this.drift * remoteUptimeSum) - localUptimeSum) / d3;
        this.predictedRemoteUptime = (this.localUptime + offset) / this.drift;
        log.info(String.format("Calibration complete. Drift: %.4g, Offset: %.4f s", new Object[]{Double.valueOf(this.drift), Double.valueOf(offset)}));
    }

    private double calculateDrift(double localUptimeDelta, double remoteUptimeDelta) {
        Preconditions.checkState(remoteUptimeDelta > 1.0E-9d);
        return localUptimeDelta / remoteUptimeDelta;
    }

    public void update() {
        UptimeCalculationResult result = calculateNewUptime(this.callable);
        double newLocalUptime = result.newLocalUptime;
        double newRemoteUptime = result.newRemoteUptime;
        double latency = result.latency;
        if (this.latencyOutlierFilter.add(latency)) {
            log.warn(String.format("Measurement latency marked as outlier. Latency: %.4f s, Median: %.4f s", new Object[]{Double.valueOf(latency), Double.valueOf(this.latencyOutlierFilter.getMedian())}));
            return;
        }
        double localUptimeDelta = newLocalUptime - this.localUptime;
        double remoteUptimeDelta = newRemoteUptime - this.measuredRemoteUptime;
        Preconditions.checkState(localUptimeDelta > 1.0E-9d);
        Preconditions.checkState(remoteUptimeDelta > 1.0E-9d);
        double localUptimeDelta2 = localUptimeDelta;
        double newDrift = (this.driftSensitivity * (localUptimeDelta / remoteUptimeDelta)) + ((1.0d - this.driftSensitivity) * this.drift);
        UptimeCalculationResult uptimeCalculationResult = result;
        double newLocalUptime2 = newLocalUptime;
        double newPredictedRemoteUptime = this.predictedRemoteUptime + (localUptimeDelta2 / (this.drift + this.errorReductionCoefficient));
        double nextPredictedRemoteUptime = newRemoteUptime + remoteUptimeDelta;
        double d = nextPredictedRemoteUptime;
        double newErrorReductionCoefficient = this.errorReductionCoefficientSensitivity * ((localUptimeDelta2 / (nextPredictedRemoteUptime - newPredictedRemoteUptime)) - newDrift);
        log.info(String.format("Latency: %.4f s, Delta ratio: %.4f, Drift: %.4g, Error reduction coefficient: %.4g, Error: %.4f s", new Object[]{Double.valueOf(latency), Double.valueOf(remoteUptimeDelta / localUptimeDelta2), Double.valueOf(newDrift), Double.valueOf(newErrorReductionCoefficient), Double.valueOf(newLocalUptime2 - toLocalUptime(newRemoteUptime))}));
        this.measuredRemoteUptime = newRemoteUptime;
        this.predictedRemoteUptime = newPredictedRemoteUptime;
        double d2 = newRemoteUptime;
        this.localUptime = newLocalUptime2;
        this.drift = newDrift;
        this.errorReductionCoefficient = newErrorReductionCoefficient;
    }

    private UptimeCalculationResult calculateNewUptime(Callable<Double> callable2) {
        double newLocalUptime = this.localUptimeProvider.getSeconds();
        try {
            double newRemoteUptime = callable2.call().doubleValue();
            double latency = this.localUptimeProvider.getSeconds() - newLocalUptime;
            return new UptimeCalculationResult(newLocalUptime + (latency / 2.0d), newRemoteUptime, latency);
        } catch (Exception e) {
            log.error(e);
            throw new RosRuntimeException((Throwable) e);
        }
    }

    public double toLocalUptime(double remoteUptime) {
        return this.localUptime + ((this.drift + this.errorReductionCoefficient) * (remoteUptime - this.predictedRemoteUptime));
    }

    /* access modifiers changed from: package-private */
    @VisibleForTesting
    public double getDrift() {
        return this.drift;
    }

    /* access modifiers changed from: package-private */
    @VisibleForTesting
    public double getErrorReductionCoefficient() {
        return this.errorReductionCoefficient;
    }
}
