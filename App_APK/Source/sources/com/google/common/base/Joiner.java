package com.google.common.base;

import com.google.common.annotations.Beta;
import com.google.common.annotations.GwtCompatible;
import java.io.IOException;
import java.util.AbstractList;
import java.util.Arrays;
import java.util.Iterator;
import java.util.Map;
import javax.annotation.CheckReturnValue;
import javax.annotation.Nullable;

@GwtCompatible
public class Joiner {
    /* access modifiers changed from: private */
    public final String separator;

    public static Joiner on(String separator2) {
        return new Joiner(separator2);
    }

    public static Joiner on(char separator2) {
        return new Joiner(String.valueOf(separator2));
    }

    private Joiner(String separator2) {
        this.separator = (String) Preconditions.checkNotNull(separator2);
    }

    private Joiner(Joiner prototype) {
        this.separator = prototype.separator;
    }

    @Deprecated
    @Beta
    public final <A extends Appendable, I extends Iterable<?> & Iterator<?>> A appendTo(A appendable, I parts) throws IOException {
        return appendTo(appendable, (Iterator<?>) (Iterator) parts);
    }

    public <A extends Appendable> A appendTo(A appendable, Iterable<?> parts) throws IOException {
        return appendTo(appendable, parts.iterator());
    }

    @Beta
    public <A extends Appendable> A appendTo(A appendable, Iterator<?> parts) throws IOException {
        Preconditions.checkNotNull(appendable);
        if (parts.hasNext()) {
            appendable.append(toString(parts.next()));
            while (parts.hasNext()) {
                appendable.append(this.separator);
                appendable.append(toString(parts.next()));
            }
        }
        return appendable;
    }

    public final <A extends Appendable> A appendTo(A appendable, Object[] parts) throws IOException {
        return appendTo(appendable, (Iterable<?>) Arrays.asList(parts));
    }

    public final <A extends Appendable> A appendTo(A appendable, @Nullable Object first, @Nullable Object second, Object... rest) throws IOException {
        return appendTo(appendable, (Iterable<?>) iterable(first, second, rest));
    }

    @Deprecated
    @Beta
    public final <I extends Iterable<?> & Iterator<?>> StringBuilder appendTo(StringBuilder builder, I parts) {
        return appendTo(builder, (Iterator<?>) (Iterator) parts);
    }

    public final StringBuilder appendTo(StringBuilder builder, Iterable<?> parts) {
        return appendTo(builder, parts.iterator());
    }

    @Beta
    public final StringBuilder appendTo(StringBuilder builder, Iterator<?> parts) {
        try {
            appendTo(builder, parts);
            return builder;
        } catch (IOException impossible) {
            throw new AssertionError(impossible);
        }
    }

    public final StringBuilder appendTo(StringBuilder builder, Object[] parts) {
        return appendTo(builder, (Iterable<?>) Arrays.asList(parts));
    }

    public final StringBuilder appendTo(StringBuilder builder, @Nullable Object first, @Nullable Object second, Object... rest) {
        return appendTo(builder, (Iterable<?>) iterable(first, second, rest));
    }

    @Deprecated
    @Beta
    public final <I extends Iterable<?> & Iterator<?>> String join(I parts) {
        return join((Iterator<?>) (Iterator) parts);
    }

    public final String join(Iterable<?> parts) {
        return join(parts.iterator());
    }

    @Beta
    public final String join(Iterator<?> parts) {
        return appendTo(new StringBuilder(), parts).toString();
    }

    public final String join(Object[] parts) {
        return join((Iterable<?>) Arrays.asList(parts));
    }

    public final String join(@Nullable Object first, @Nullable Object second, Object... rest) {
        return join((Iterable<?>) iterable(first, second, rest));
    }

    @CheckReturnValue
    public Joiner useForNull(final String nullText) {
        Preconditions.checkNotNull(nullText);
        return new Joiner(this) {
            /* access modifiers changed from: package-private */
            public CharSequence toString(Object part) {
                return part == null ? nullText : Joiner.this.toString(part);
            }

            public Joiner useForNull(String nullText) {
                Preconditions.checkNotNull(nullText);
                throw new UnsupportedOperationException("already specified useForNull");
            }

            public Joiner skipNulls() {
                throw new UnsupportedOperationException("already specified useForNull");
            }
        };
    }

    @CheckReturnValue
    public Joiner skipNulls() {
        return new Joiner(this) {
            public <A extends Appendable> A appendTo(A appendable, Iterator<?> parts) throws IOException {
                Preconditions.checkNotNull(appendable, "appendable");
                Preconditions.checkNotNull(parts, "parts");
                while (true) {
                    if (parts.hasNext()) {
                        Object part = parts.next();
                        if (part != null) {
                            appendable.append(Joiner.this.toString(part));
                            break;
                        }
                    } else {
                        break;
                    }
                }
                while (parts.hasNext()) {
                    Object part2 = parts.next();
                    if (part2 != null) {
                        appendable.append(Joiner.this.separator);
                        appendable.append(Joiner.this.toString(part2));
                    }
                }
                return appendable;
            }

            public Joiner useForNull(String nullText) {
                Preconditions.checkNotNull(nullText);
                throw new UnsupportedOperationException("already specified skipNulls");
            }

            public MapJoiner withKeyValueSeparator(String kvs) {
                Preconditions.checkNotNull(kvs);
                throw new UnsupportedOperationException("can't use .skipNulls() with maps");
            }
        };
    }

    @CheckReturnValue
    public MapJoiner withKeyValueSeparator(String keyValueSeparator) {
        return new MapJoiner(keyValueSeparator);
    }

    public static final class MapJoiner {
        private final Joiner joiner;
        private final String keyValueSeparator;

        private MapJoiner(Joiner joiner2, String keyValueSeparator2) {
            this.joiner = joiner2;
            this.keyValueSeparator = (String) Preconditions.checkNotNull(keyValueSeparator2);
        }

        public <A extends Appendable> A appendTo(A appendable, Map<?, ?> map) throws IOException {
            return appendTo(appendable, (Iterable<? extends Map.Entry<?, ?>>) map.entrySet());
        }

        public StringBuilder appendTo(StringBuilder builder, Map<?, ?> map) {
            return appendTo(builder, (Iterable<? extends Map.Entry<?, ?>>) map.entrySet());
        }

        public String join(Map<?, ?> map) {
            return join((Iterable<? extends Map.Entry<?, ?>>) map.entrySet());
        }

        @Deprecated
        @Beta
        public <A extends Appendable, I extends Iterable<? extends Map.Entry<?, ?>> & Iterator<? extends Map.Entry<?, ?>>> A appendTo(A appendable, I entries) throws IOException {
            return appendTo(appendable, (Iterator<? extends Map.Entry<?, ?>>) entries);
        }

        @Beta
        public <A extends Appendable> A appendTo(A appendable, Iterable<? extends Map.Entry<?, ?>> entries) throws IOException {
            return appendTo(appendable, entries.iterator());
        }

        @Beta
        public <A extends Appendable> A appendTo(A appendable, Iterator<? extends Map.Entry<?, ?>> parts) throws IOException {
            Preconditions.checkNotNull(appendable);
            if (parts.hasNext()) {
                Map.Entry<?, ?> entry = (Map.Entry) parts.next();
                appendable.append(this.joiner.toString(entry.getKey()));
                appendable.append(this.keyValueSeparator);
                appendable.append(this.joiner.toString(entry.getValue()));
                while (parts.hasNext()) {
                    appendable.append(this.joiner.separator);
                    Map.Entry<?, ?> e = (Map.Entry) parts.next();
                    appendable.append(this.joiner.toString(e.getKey()));
                    appendable.append(this.keyValueSeparator);
                    appendable.append(this.joiner.toString(e.getValue()));
                }
            }
            return appendable;
        }

        @Deprecated
        @Beta
        public <I extends Iterable<? extends Map.Entry<?, ?>> & Iterator<? extends Map.Entry<?, ?>>> StringBuilder appendTo(StringBuilder builder, I entries) throws IOException {
            return appendTo(builder, (Iterator<? extends Map.Entry<?, ?>>) entries);
        }

        @Beta
        public StringBuilder appendTo(StringBuilder builder, Iterable<? extends Map.Entry<?, ?>> entries) {
            return appendTo(builder, entries.iterator());
        }

        @Beta
        public StringBuilder appendTo(StringBuilder builder, Iterator<? extends Map.Entry<?, ?>> entries) {
            try {
                appendTo(builder, entries);
                return builder;
            } catch (IOException impossible) {
                throw new AssertionError(impossible);
            }
        }

        @Deprecated
        @Beta
        public <I extends Iterable<? extends Map.Entry<?, ?>> & Iterator<? extends Map.Entry<?, ?>>> String join(I entries) throws IOException {
            return join((Iterator<? extends Map.Entry<?, ?>>) entries);
        }

        @Beta
        public String join(Iterable<? extends Map.Entry<?, ?>> entries) {
            return join(entries.iterator());
        }

        @Beta
        public String join(Iterator<? extends Map.Entry<?, ?>> entries) {
            return appendTo(new StringBuilder(), entries).toString();
        }

        @CheckReturnValue
        public MapJoiner useForNull(String nullText) {
            return new MapJoiner(this.joiner.useForNull(nullText), this.keyValueSeparator);
        }
    }

    /* access modifiers changed from: package-private */
    public CharSequence toString(Object part) {
        Preconditions.checkNotNull(part);
        return part instanceof CharSequence ? (CharSequence) part : part.toString();
    }

    private static Iterable<Object> iterable(final Object first, final Object second, final Object[] rest) {
        Preconditions.checkNotNull(rest);
        return new AbstractList<Object>() {
            public int size() {
                return rest.length + 2;
            }

            public Object get(int index) {
                switch (index) {
                    case 0:
                        return first;
                    case 1:
                        return second;
                    default:
                        return rest[index - 2];
                }
            }
        };
    }
}
