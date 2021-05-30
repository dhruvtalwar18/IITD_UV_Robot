package org.jboss.netty.channel;

import java.util.ArrayList;
import java.util.List;
import org.xbill.DNS.TTL;

public class AdaptiveReceiveBufferSizePredictor implements ReceiveBufferSizePredictor {
    static final int DEFAULT_INITIAL = 1024;
    static final int DEFAULT_MAXIMUM = 65536;
    static final int DEFAULT_MINIMUM = 64;
    private static final int INDEX_DECREMENT = 1;
    private static final int INDEX_INCREMENT = 4;
    private static final int[] SIZE_TABLE;
    private boolean decreaseNow;
    private int index;
    private final int maxIndex;
    private final int minIndex;
    private int nextReceiveBufferSize;

    static {
        int j;
        List<Integer> sizeTable = new ArrayList<>();
        for (int i = 1; i <= 8; i++) {
            sizeTable.add(Integer.valueOf(i));
        }
        int i2 = 4;
        while (true) {
            j = 0;
            if (i2 >= 32) {
                break;
            }
            long v = 1 << i2;
            long inc = v >>> 4;
            long v2 = v - (inc << 3);
            while (true) {
                int j2 = j;
                if (j2 >= 8) {
                    break;
                }
                v2 += inc;
                if (v2 > TTL.MAX_VALUE) {
                    sizeTable.add(Integer.MAX_VALUE);
                } else {
                    sizeTable.add(Integer.valueOf((int) v2));
                }
                j = j2 + 1;
            }
            i2++;
        }
        SIZE_TABLE = new int[sizeTable.size()];
        while (true) {
            int i3 = j;
            if (i3 < SIZE_TABLE.length) {
                SIZE_TABLE[i3] = sizeTable.get(i3).intValue();
                j = i3 + 1;
            } else {
                return;
            }
        }
    }

    private static int getSizeTableIndex(int size) {
        if (size <= 16) {
            return size - 1;
        }
        int bits = 0;
        int v = size;
        do {
            v >>>= 1;
            bits++;
        } while (v != 0);
        int baseIdx = bits << 3;
        int endIdx = baseIdx - 25;
        for (int i = baseIdx - 18; i >= endIdx; i--) {
            if (size >= SIZE_TABLE[i]) {
                return i;
            }
        }
        throw new Error("shouldn't reach here; please file a bug report.");
    }

    public AdaptiveReceiveBufferSizePredictor() {
        this(64, 1024, 65536);
    }

    public AdaptiveReceiveBufferSizePredictor(int minimum, int initial, int maximum) {
        if (minimum <= 0) {
            throw new IllegalArgumentException("minimum: " + minimum);
        } else if (initial < minimum) {
            throw new IllegalArgumentException("initial: " + initial);
        } else if (maximum >= initial) {
            int minIndex2 = getSizeTableIndex(minimum);
            if (SIZE_TABLE[minIndex2] < minimum) {
                this.minIndex = minIndex2 + 1;
            } else {
                this.minIndex = minIndex2;
            }
            int maxIndex2 = getSizeTableIndex(maximum);
            if (SIZE_TABLE[maxIndex2] > maximum) {
                this.maxIndex = maxIndex2 - 1;
            } else {
                this.maxIndex = maxIndex2;
            }
            this.index = getSizeTableIndex(initial);
            this.nextReceiveBufferSize = SIZE_TABLE[this.index];
        } else {
            throw new IllegalArgumentException("maximum: " + maximum);
        }
    }

    public int nextReceiveBufferSize() {
        return this.nextReceiveBufferSize;
    }

    public void previousReceiveBufferSize(int previousReceiveBufferSize) {
        if (previousReceiveBufferSize <= SIZE_TABLE[Math.max(0, (this.index - 1) - 1)]) {
            if (this.decreaseNow) {
                this.index = Math.max(this.index - 1, this.minIndex);
                this.nextReceiveBufferSize = SIZE_TABLE[this.index];
                this.decreaseNow = false;
                return;
            }
            this.decreaseNow = true;
        } else if (previousReceiveBufferSize >= this.nextReceiveBufferSize) {
            this.index = Math.min(this.index + 4, this.maxIndex);
            this.nextReceiveBufferSize = SIZE_TABLE[this.index];
            this.decreaseNow = false;
        }
    }
}
