package com.google.common.util.concurrent;

import com.google.common.annotations.Beta;

@Beta
public interface AsyncFunction<I, O> {
    ListenableFuture<O> apply(I i) throws Exception;
}
