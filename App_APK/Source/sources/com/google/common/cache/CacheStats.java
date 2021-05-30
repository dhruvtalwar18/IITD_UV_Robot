package com.google.common.cache;

import com.google.common.annotations.Beta;
import com.google.common.annotations.GwtCompatible;
import com.google.common.base.Objects;
import com.google.common.base.Preconditions;
import javax.annotation.Nullable;
import org.bytedeco.javacpp.opencv_stitching;

@GwtCompatible
@Beta
public final class CacheStats {
    private final long evictionCount;
    private final long hitCount;
    private final long loadExceptionCount;
    private final long loadSuccessCount;
    private final long missCount;
    private final long totalLoadTime;

    public CacheStats(long hitCount2, long missCount2, long loadSuccessCount2, long loadExceptionCount2, long totalLoadTime2, long evictionCount2) {
        long j = hitCount2;
        long j2 = missCount2;
        long j3 = loadSuccessCount2;
        long j4 = loadExceptionCount2;
        long j5 = totalLoadTime2;
        long j6 = evictionCount2;
        boolean z = false;
        Preconditions.checkArgument(j >= 0);
        Preconditions.checkArgument(j2 >= 0);
        Preconditions.checkArgument(j3 >= 0);
        Preconditions.checkArgument(j4 >= 0);
        Preconditions.checkArgument(j5 >= 0);
        Preconditions.checkArgument(j6 >= 0 ? true : z);
        this.hitCount = j;
        this.missCount = j2;
        this.loadSuccessCount = j3;
        this.loadExceptionCount = j4;
        this.totalLoadTime = j5;
        this.evictionCount = j6;
    }

    public long requestCount() {
        return this.hitCount + this.missCount;
    }

    public long hitCount() {
        return this.hitCount;
    }

    public double hitRate() {
        long requestCount = requestCount();
        if (requestCount == 0) {
            return 1.0d;
        }
        double d = (double) this.hitCount;
        double d2 = (double) requestCount;
        Double.isNaN(d);
        Double.isNaN(d2);
        return d / d2;
    }

    public long missCount() {
        return this.missCount;
    }

    public double missRate() {
        long requestCount = requestCount();
        if (requestCount == 0) {
            return opencv_stitching.Stitcher.ORIG_RESOL;
        }
        double d = (double) this.missCount;
        double d2 = (double) requestCount;
        Double.isNaN(d);
        Double.isNaN(d2);
        return d / d2;
    }

    public long loadCount() {
        return this.loadSuccessCount + this.loadExceptionCount;
    }

    public long loadSuccessCount() {
        return this.loadSuccessCount;
    }

    public long loadExceptionCount() {
        return this.loadExceptionCount;
    }

    public double loadExceptionRate() {
        long totalLoadCount = this.loadSuccessCount + this.loadExceptionCount;
        if (totalLoadCount == 0) {
            return opencv_stitching.Stitcher.ORIG_RESOL;
        }
        double d = (double) this.loadExceptionCount;
        double d2 = (double) totalLoadCount;
        Double.isNaN(d);
        Double.isNaN(d2);
        return d / d2;
    }

    public long totalLoadTime() {
        return this.totalLoadTime;
    }

    public double averageLoadPenalty() {
        long totalLoadCount = this.loadSuccessCount + this.loadExceptionCount;
        if (totalLoadCount == 0) {
            return opencv_stitching.Stitcher.ORIG_RESOL;
        }
        double d = (double) this.totalLoadTime;
        double d2 = (double) totalLoadCount;
        Double.isNaN(d);
        Double.isNaN(d2);
        return d / d2;
    }

    public long evictionCount() {
        return this.evictionCount;
    }

    public CacheStats minus(CacheStats other) {
        CacheStats cacheStats = other;
        long max = Math.max(0, this.hitCount - cacheStats.hitCount);
        long max2 = Math.max(0, this.missCount - cacheStats.missCount);
        long max3 = Math.max(0, this.loadSuccessCount - cacheStats.loadSuccessCount);
        long max4 = Math.max(0, this.loadExceptionCount - cacheStats.loadExceptionCount);
        return new CacheStats(max, max2, max3, max4, Math.max(0, this.totalLoadTime - cacheStats.totalLoadTime), Math.max(0, this.evictionCount - cacheStats.evictionCount));
    }

    public CacheStats plus(CacheStats other) {
        CacheStats cacheStats = other;
        return new CacheStats(cacheStats.hitCount + this.hitCount, cacheStats.missCount + this.missCount, cacheStats.loadSuccessCount + this.loadSuccessCount, cacheStats.loadExceptionCount + this.loadExceptionCount, cacheStats.totalLoadTime + this.totalLoadTime, this.evictionCount + cacheStats.evictionCount);
    }

    public int hashCode() {
        return Objects.hashCode(Long.valueOf(this.hitCount), Long.valueOf(this.missCount), Long.valueOf(this.loadSuccessCount), Long.valueOf(this.loadExceptionCount), Long.valueOf(this.totalLoadTime), Long.valueOf(this.evictionCount));
    }

    public boolean equals(@Nullable Object object) {
        if (!(object instanceof CacheStats)) {
            return false;
        }
        CacheStats other = (CacheStats) object;
        if (this.hitCount == other.hitCount && this.missCount == other.missCount && this.loadSuccessCount == other.loadSuccessCount && this.loadExceptionCount == other.loadExceptionCount && this.totalLoadTime == other.totalLoadTime && this.evictionCount == other.evictionCount) {
            return true;
        }
        return false;
    }

    public String toString() {
        return Objects.toStringHelper((Object) this).add("hitCount", this.hitCount).add("missCount", this.missCount).add("loadSuccessCount", this.loadSuccessCount).add("loadExceptionCount", this.loadExceptionCount).add("totalLoadTime", this.totalLoadTime).add("evictionCount", this.evictionCount).toString();
    }
}
