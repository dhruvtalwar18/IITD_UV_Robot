package com.google.common.collect;

@Deprecated
public class AsynchronousComputationException extends ComputationException {
    private static final long serialVersionUID = 0;

    public AsynchronousComputationException(Throwable cause) {
        super(cause);
    }
}
