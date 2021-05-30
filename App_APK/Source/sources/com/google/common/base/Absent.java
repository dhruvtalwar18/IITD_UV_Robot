package com.google.common.base;

import com.google.common.annotations.GwtCompatible;
import java.util.Collections;
import java.util.Set;
import javax.annotation.Nullable;

@GwtCompatible
final class Absent extends Optional<Object> {
    static final Absent INSTANCE = new Absent();
    private static final long serialVersionUID = 0;

    Absent() {
    }

    public boolean isPresent() {
        return false;
    }

    public Object get() {
        throw new IllegalStateException("value is absent");
    }

    public Object or(Object defaultValue) {
        return Preconditions.checkNotNull(defaultValue, "use orNull() instead of or(null)");
    }

    public Optional<Object> or(Optional<?> secondChoice) {
        return (Optional) Preconditions.checkNotNull(secondChoice);
    }

    public Object or(Supplier<?> supplier) {
        return Preconditions.checkNotNull(supplier.get(), "use orNull() instead of a Supplier that returns null");
    }

    @Nullable
    public Object orNull() {
        return null;
    }

    public Set<Object> asSet() {
        return Collections.emptySet();
    }

    public <V> Optional<V> transform(Function<Object, V> function) {
        Preconditions.checkNotNull(function);
        return Optional.absent();
    }

    public boolean equals(@Nullable Object object) {
        return object == this;
    }

    public int hashCode() {
        return 1502476572;
    }

    public String toString() {
        return "Optional.absent()";
    }

    private Object readResolve() {
        return INSTANCE;
    }
}
