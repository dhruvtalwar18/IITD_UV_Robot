package org.bytedeco.javacpp;

public abstract class FunctionPointer extends Pointer {
    protected FunctionPointer() {
    }

    protected FunctionPointer(Pointer p) {
        super(p);
    }
}
