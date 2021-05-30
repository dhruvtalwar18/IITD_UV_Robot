package org.ros.time;

import com.google.common.base.Preconditions;
import com.google.common.collect.Lists;
import java.io.IOException;
import java.net.InetAddress;
import java.util.List;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.ScheduledFuture;
import java.util.concurrent.TimeUnit;
import org.apache.commons.logging.Log;
import org.apache.commons.logging.LogFactory;
import org.apache.commons.net.ntp.NTPUDPClient;
import org.apache.commons.net.ntp.TimeInfo;
import org.ros.math.CollectionMath;
import org.ros.message.Duration;
import org.ros.message.Time;

public class NtpTimeProvider implements TimeProvider {
    private static final boolean DEBUG = false;
    /* access modifiers changed from: private */
    public static final Log log = LogFactory.getLog(NtpTimeProvider.class);
    private final InetAddress host;
    private final NTPUDPClient ntpClient;
    private long offset;
    private int sampleSize = 11;
    private final ScheduledExecutorService scheduledExecutorService;
    private ScheduledFuture<?> scheduledFuture;
    private final WallTimeProvider wallTimeProvider;

    public NtpTimeProvider(InetAddress host2, ScheduledExecutorService scheduledExecutorService2) {
        this.host = host2;
        this.scheduledExecutorService = scheduledExecutorService2;
        this.wallTimeProvider = new WallTimeProvider();
        this.ntpClient = new NTPUDPClient();
        this.ntpClient.setDefaultTimeout(500);
        this.offset = 0;
        this.scheduledFuture = null;
    }

    public void updateTime() throws IOException {
        List<Long> offsets = Lists.newArrayList();
        int failures = 0;
        for (int i = 0; i < this.sampleSize; i++) {
            try {
                offsets.add(Long.valueOf(computeOffset()));
            } catch (IOException e) {
                failures++;
                if (failures > this.sampleSize / 2) {
                    throw e;
                }
            }
        }
        this.offset = ((Long) CollectionMath.median(offsets)).longValue();
        log.info(String.format("NTP time offset: %d ms", new Object[]{Long.valueOf(this.offset)}));
    }

    private long computeOffset() throws IOException {
        try {
            TimeInfo time = this.ntpClient.getTime(this.host);
            time.computeDetails();
            return time.getOffset().longValue();
        } catch (IOException e) {
            throw e;
        }
    }

    public void startPeriodicUpdates(long period, TimeUnit unit) {
        this.scheduledFuture = this.scheduledExecutorService.scheduleAtFixedRate(new Runnable() {
            public void run() {
                try {
                    NtpTimeProvider.this.updateTime();
                } catch (IOException e) {
                    NtpTimeProvider.log.error("Periodic NTP update failed.", e);
                }
            }
        }, 0, period, unit);
    }

    public void stopPeriodicUpdates() {
        Preconditions.checkNotNull(this.scheduledFuture);
        this.scheduledFuture.cancel(true);
        this.scheduledFuture = null;
    }

    public Time getCurrentTime() {
        return this.wallTimeProvider.getCurrentTime().add(Duration.fromMillis(this.offset));
    }

    public void setUpdateTimeSampleSize(int sampleSize2) {
        Preconditions.checkArgument(sampleSize2 > 0);
        this.sampleSize = sampleSize2;
    }
}
