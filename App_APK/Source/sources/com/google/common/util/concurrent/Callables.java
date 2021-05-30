package com.google.common.util.concurrent;

import java.util.concurrent.Callable;
import javax.annotation.Nullable;

public final class Callables {
    private Callables() {
    }

    public static <T> Callable<T> returning(@Nullable final T value) {
        return new Callable<T>() {
            public T call() {
                return value;
            }
        };
    }
}
