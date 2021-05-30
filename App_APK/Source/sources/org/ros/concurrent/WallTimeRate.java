package org.ros.concurrent;

public class WallTimeRate implements Rate {
    private final long delay;
    private long time = 0;

    public WallTimeRate(int hz) {
        this.delay = (long) (1000 / hz);
    }

    public void sleep() {
        long delta = System.currentTimeMillis() - this.time;
        while (delta < this.delay) {
            try {
                Thread.sleep(this.delay - delta);
                delta = System.currentTimeMillis() - this.time;
            } catch (InterruptedException e) {
            }
        }
        this.time = System.currentTimeMillis();
    }
}
