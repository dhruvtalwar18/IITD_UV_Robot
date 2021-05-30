package org.jboss.netty.util.internal;

import java.util.Random;

final class ThreadLocalRandom extends Random {
    private static final long addend = 11;
    private static final ThreadLocal<ThreadLocalRandom> localRandom = new ThreadLocal<ThreadLocalRandom>() {
        /* access modifiers changed from: protected */
        public ThreadLocalRandom initialValue() {
            return new ThreadLocalRandom();
        }
    };
    private static final long mask = 281474976710655L;
    private static final long multiplier = 25214903917L;
    private static final long serialVersionUID = -5851777807851030925L;
    private boolean initialized;
    private long pad0;
    private long pad1;
    private long pad2;
    private long pad3;
    private long pad4;
    private long pad5;
    private long pad6;
    private long pad7;
    private long rnd;

    ThreadLocalRandom() {
    }

    static ThreadLocalRandom current() {
        return localRandom.get();
    }

    public void setSeed(long seed) {
        if (!this.initialized) {
            this.initialized = true;
            this.rnd = (multiplier ^ seed) & mask;
            return;
        }
        throw new UnsupportedOperationException();
    }

    /* access modifiers changed from: protected */
    public int next(int bits) {
        this.rnd = ((this.rnd * multiplier) + addend) & mask;
        return (int) (this.rnd >>> (48 - bits));
    }
}
