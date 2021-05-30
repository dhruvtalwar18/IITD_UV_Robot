package com.google.common.collect;

import com.google.common.annotations.GwtCompatible;
import com.google.common.base.Preconditions;
import com.google.common.collect.ImmutableMap;
import java.util.Map;
import javax.annotation.Nullable;
import javax.annotation.concurrent.Immutable;

@GwtCompatible(emulated = true, serializable = true)
final class RegularImmutableMap<K, V> extends ImmutableMap<K, V> {
    private static final long serialVersionUID = 0;
    /* access modifiers changed from: private */
    public final transient LinkedEntry<K, V>[] entries;
    private final transient int keySetHashCode;
    private final transient int mask;
    private final transient LinkedEntry<K, V>[] table;

    private interface LinkedEntry<K, V> extends Map.Entry<K, V> {
        @Nullable
        LinkedEntry<K, V> next();
    }

    RegularImmutableMap(Map.Entry<?, ?>... immutableEntries) {
        int size = immutableEntries.length;
        this.entries = createEntryArray(size);
        int tableSize = chooseTableSize(size);
        this.table = createEntryArray(tableSize);
        this.mask = tableSize - 1;
        int keySetHashCodeMutable = 0;
        for (int entryIndex = 0; entryIndex < size; entryIndex++) {
            Map.Entry<K, V> entry = immutableEntries[entryIndex];
            K key = entry.getKey();
            int keyHashCode = key.hashCode();
            keySetHashCodeMutable += keyHashCode;
            int tableIndex = Hashing.smear(keyHashCode) & this.mask;
            LinkedEntry<K, V> existing = this.table[tableIndex];
            LinkedEntry<K, V> linkedEntry = newLinkedEntry(key, entry.getValue(), existing);
            this.table[tableIndex] = linkedEntry;
            this.entries[entryIndex] = linkedEntry;
            while (existing != null) {
                Preconditions.checkArgument(!key.equals(existing.getKey()), "duplicate key: %s", key);
                existing = existing.next();
            }
        }
        this.keySetHashCode = keySetHashCodeMutable;
    }

    private static int chooseTableSize(int size) {
        int tableSize = Integer.highestOneBit(size) << 1;
        Preconditions.checkArgument(tableSize > 0, "table too large: %s", Integer.valueOf(size));
        return tableSize;
    }

    private LinkedEntry<K, V>[] createEntryArray(int size) {
        return new LinkedEntry[size];
    }

    private static <K, V> LinkedEntry<K, V> newLinkedEntry(K key, V value, @Nullable LinkedEntry<K, V> next) {
        return next == null ? new TerminalEntry(key, value) : new NonTerminalEntry(key, value, next);
    }

    @Immutable
    private static final class NonTerminalEntry<K, V> extends ImmutableEntry<K, V> implements LinkedEntry<K, V> {
        final LinkedEntry<K, V> next;

        NonTerminalEntry(K key, V value, LinkedEntry<K, V> next2) {
            super(key, value);
            this.next = next2;
        }

        public LinkedEntry<K, V> next() {
            return this.next;
        }
    }

    @Immutable
    private static final class TerminalEntry<K, V> extends ImmutableEntry<K, V> implements LinkedEntry<K, V> {
        TerminalEntry(K key, V value) {
            super(key, value);
        }

        @Nullable
        public LinkedEntry<K, V> next() {
            return null;
        }
    }

    public V get(@Nullable Object key) {
        if (key == null) {
            return null;
        }
        for (LinkedEntry<K, V> entry = this.table[Hashing.smear(key.hashCode()) & this.mask]; entry != null; entry = entry.next()) {
            if (key.equals(entry.getKey())) {
                return entry.getValue();
            }
        }
        return null;
    }

    public int size() {
        return this.entries.length;
    }

    public boolean isEmpty() {
        return false;
    }

    public boolean containsValue(@Nullable Object value) {
        if (value == null) {
            return false;
        }
        for (Map.Entry<K, V> entry : this.entries) {
            if (entry.getValue().equals(value)) {
                return true;
            }
        }
        return false;
    }

    /* access modifiers changed from: package-private */
    public boolean isPartialView() {
        return false;
    }

    /* access modifiers changed from: package-private */
    public ImmutableSet<Map.Entry<K, V>> createEntrySet() {
        return new EntrySet();
    }

    private class EntrySet extends ImmutableMap.EntrySet {
        private EntrySet() {
            super();
        }

        public UnmodifiableIterator<Map.Entry<K, V>> iterator() {
            return asList().iterator();
        }

        /* access modifiers changed from: package-private */
        public ImmutableList<Map.Entry<K, V>> createAsList() {
            return new RegularImmutableList(RegularImmutableMap.this.entries);
        }
    }

    /* access modifiers changed from: package-private */
    public ImmutableSet<K> createKeySet() {
        return new ImmutableMap.KeySet(this.keySetHashCode);
    }

    public String toString() {
        StringBuilder result = Collections2.newStringBuilderForCollection(size()).append('{');
        Collections2.STANDARD_JOINER.appendTo(result, (Object[]) this.entries);
        result.append('}');
        return result.toString();
    }
}
