package com.google.common.collect;

import com.google.common.annotations.GwtCompatible;
import java.util.Map;
import javax.annotation.Nullable;

@GwtCompatible(emulated = true, serializable = true)
final class SingletonImmutableMap<K, V> extends ImmutableMap<K, V> {
    private transient Map.Entry<K, V> entry;
    final transient K singleKey;
    final transient V singleValue;

    SingletonImmutableMap(K singleKey2, V singleValue2) {
        this.singleKey = singleKey2;
        this.singleValue = singleValue2;
    }

    SingletonImmutableMap(Map.Entry<K, V> entry2) {
        this.entry = entry2;
        this.singleKey = entry2.getKey();
        this.singleValue = entry2.getValue();
    }

    private Map.Entry<K, V> entry() {
        Map.Entry<K, V> e = this.entry;
        if (e != null) {
            return e;
        }
        Map.Entry<K, V> immutableEntry = Maps.immutableEntry(this.singleKey, this.singleValue);
        this.entry = immutableEntry;
        return immutableEntry;
    }

    public V get(@Nullable Object key) {
        if (this.singleKey.equals(key)) {
            return this.singleValue;
        }
        return null;
    }

    public int size() {
        return 1;
    }

    public boolean isEmpty() {
        return false;
    }

    public boolean containsKey(@Nullable Object key) {
        return this.singleKey.equals(key);
    }

    public boolean containsValue(@Nullable Object value) {
        return this.singleValue.equals(value);
    }

    /* access modifiers changed from: package-private */
    public boolean isPartialView() {
        return false;
    }

    /* access modifiers changed from: package-private */
    public ImmutableSet<Map.Entry<K, V>> createEntrySet() {
        return ImmutableSet.of(entry());
    }

    /* access modifiers changed from: package-private */
    public ImmutableSet<K> createKeySet() {
        return ImmutableSet.of(this.singleKey);
    }

    /* access modifiers changed from: package-private */
    public ImmutableCollection<V> createValues() {
        return ImmutableList.of(this.singleValue);
    }

    public boolean equals(@Nullable Object object) {
        if (object == this) {
            return true;
        }
        if (!(object instanceof Map)) {
            return false;
        }
        Map<?, ?> that = (Map) object;
        if (that.size() != 1) {
            return false;
        }
        Map.Entry<?, ?> entry2 = that.entrySet().iterator().next();
        if (!this.singleKey.equals(entry2.getKey()) || !this.singleValue.equals(entry2.getValue())) {
            return false;
        }
        return true;
    }

    public int hashCode() {
        return this.singleKey.hashCode() ^ this.singleValue.hashCode();
    }

    public String toString() {
        return '{' + this.singleKey.toString() + '=' + this.singleValue.toString() + '}';
    }
}
