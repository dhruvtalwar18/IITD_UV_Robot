package com.google.common.base;

import com.google.common.annotations.Beta;
import com.google.common.annotations.GwtCompatible;
import java.util.Arrays;
import java.util.LinkedList;
import java.util.List;
import javax.annotation.Nullable;

@GwtCompatible
public final class Objects {
    private Objects() {
    }

    public static boolean equal(@Nullable Object a, @Nullable Object b) {
        return a == b || (a != null && a.equals(b));
    }

    public static int hashCode(@Nullable Object... objects) {
        return Arrays.hashCode(objects);
    }

    public static ToStringHelper toStringHelper(Object self) {
        return new ToStringHelper(simpleName(self.getClass()));
    }

    public static ToStringHelper toStringHelper(Class<?> clazz) {
        return new ToStringHelper(simpleName(clazz));
    }

    public static ToStringHelper toStringHelper(String className) {
        return new ToStringHelper(className);
    }

    private static String simpleName(Class<?> clazz) {
        String name = clazz.getName().replaceAll("\\$[0-9]+", "\\$");
        int start = name.lastIndexOf(36);
        if (start == -1) {
            start = name.lastIndexOf(46);
        }
        return name.substring(start + 1);
    }

    public static <T> T firstNonNull(@Nullable T first, @Nullable T second) {
        return first != null ? first : Preconditions.checkNotNull(second);
    }

    public static final class ToStringHelper {
        private final String className;
        private boolean omitNullValues;
        private final List<ValueHolder> valueHolders;

        private ToStringHelper(String className2) {
            this.valueHolders = new LinkedList();
            this.omitNullValues = false;
            this.className = (String) Preconditions.checkNotNull(className2);
        }

        @Beta
        public ToStringHelper omitNullValues() {
            this.omitNullValues = true;
            return this;
        }

        public ToStringHelper add(String name, @Nullable Object value) {
            Preconditions.checkNotNull(name);
            StringBuilder sb = addHolder(value).builder;
            sb.append(name);
            sb.append('=');
            sb.append(value);
            return this;
        }

        public ToStringHelper add(String name, boolean value) {
            checkNameAndAppend(name).append(value);
            return this;
        }

        public ToStringHelper add(String name, char value) {
            checkNameAndAppend(name).append(value);
            return this;
        }

        public ToStringHelper add(String name, double value) {
            checkNameAndAppend(name).append(value);
            return this;
        }

        public ToStringHelper add(String name, float value) {
            checkNameAndAppend(name).append(value);
            return this;
        }

        public ToStringHelper add(String name, int value) {
            checkNameAndAppend(name).append(value);
            return this;
        }

        public ToStringHelper add(String name, long value) {
            checkNameAndAppend(name).append(value);
            return this;
        }

        private StringBuilder checkNameAndAppend(String name) {
            Preconditions.checkNotNull(name);
            StringBuilder sb = addHolder().builder;
            sb.append(name);
            sb.append('=');
            return sb;
        }

        public ToStringHelper addValue(@Nullable Object value) {
            addHolder(value).builder.append(value);
            return this;
        }

        public ToStringHelper addValue(boolean value) {
            addHolder().builder.append(value);
            return this;
        }

        public ToStringHelper addValue(char value) {
            addHolder().builder.append(value);
            return this;
        }

        public ToStringHelper addValue(double value) {
            addHolder().builder.append(value);
            return this;
        }

        public ToStringHelper addValue(float value) {
            addHolder().builder.append(value);
            return this;
        }

        public ToStringHelper addValue(int value) {
            addHolder().builder.append(value);
            return this;
        }

        public ToStringHelper addValue(long value) {
            addHolder().builder.append(value);
            return this;
        }

        public String toString() {
            boolean omitNullValuesSnapshot = this.omitNullValues;
            boolean needsSeparator = false;
            StringBuilder sb = new StringBuilder(32);
            sb.append(this.className);
            StringBuilder builder = sb.append('{');
            for (ValueHolder valueHolder : this.valueHolders) {
                if (!omitNullValuesSnapshot || !valueHolder.isNull) {
                    if (needsSeparator) {
                        builder.append(", ");
                    } else {
                        needsSeparator = true;
                    }
                    builder.append(valueHolder.builder);
                }
            }
            builder.append('}');
            return builder.toString();
        }

        private ValueHolder addHolder() {
            ValueHolder valueHolder = new ValueHolder();
            this.valueHolders.add(valueHolder);
            return valueHolder;
        }

        private ValueHolder addHolder(@Nullable Object value) {
            ValueHolder valueHolder = addHolder();
            valueHolder.isNull = value == null;
            return valueHolder;
        }

        private static final class ValueHolder {
            final StringBuilder builder;
            boolean isNull;

            private ValueHolder() {
                this.builder = new StringBuilder();
            }
        }
    }
}
