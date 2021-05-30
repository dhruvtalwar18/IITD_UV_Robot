package org.ros.concurrent;

import com.google.common.base.Preconditions;
import java.util.concurrent.CountDownLatch;
import java.util.concurrent.TimeUnit;

public class Holder<T> {
    private final CountDownLatch latch = new CountDownLatch(1);
    private T value = null;

    public static <T> Holder<T> newEmpty() {
        return new Holder<>();
    }

    private Holder() {
    }

    public T set(T value2) {
        Preconditions.checkState(this.value == null);
        this.value = value2;
        this.latch.countDown();
        return value2;
    }

    public T get() {
        Preconditions.checkNotNull(this.value);
        return this.value;
    }

    public void await() throws InterruptedException {
        this.latch.await();
    }

    public boolean await(long timeout, TimeUnit unit) throws InterruptedException {
        return this.latch.await(timeout, unit);
    }

    public boolean equals(Object obj) {
        if (this == obj) {
            return true;
        }
        return false;
    }
}
