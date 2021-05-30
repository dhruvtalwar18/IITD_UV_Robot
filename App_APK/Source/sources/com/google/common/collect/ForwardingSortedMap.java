package com.google.common.collect;

import com.google.common.annotations.Beta;
import com.google.common.annotations.GwtCompatible;
import java.util.Comparator;
import java.util.Iterator;
import java.util.Map;
import java.util.NoSuchElementException;
import java.util.SortedMap;
import javax.annotation.Nullable;

@GwtCompatible
public abstract class ForwardingSortedMap<K, V> extends ForwardingMap<K, V> implements SortedMap<K, V> {
    /* access modifiers changed from: protected */
    public abstract SortedMap<K, V> delegate();

    protected ForwardingSortedMap() {
    }

    public Comparator<? super K> comparator() {
        return delegate().comparator();
    }

    public K firstKey() {
        return delegate().firstKey();
    }

    public SortedMap<K, V> headMap(K toKey) {
        return delegate().headMap(toKey);
    }

    public K lastKey() {
        return delegate().lastKey();
    }

    public SortedMap<K, V> subMap(K fromKey, K toKey) {
        return delegate().subMap(fromKey, toKey);
    }

    public SortedMap<K, V> tailMap(K fromKey) {
        return delegate().tailMap(fromKey);
    }

    private int unsafeCompare(Object k1, Object k2) {
        Comparator<? super K> comparator = comparator();
        if (comparator == null) {
            return ((Comparable) k1).compareTo(k2);
        }
        return comparator.compare(k1, k2);
    }

    /* access modifiers changed from: protected */
    @Beta
    public boolean standardContainsKey(@Nullable Object key) {
        try {
            if (unsafeCompare(tailMap(key).firstKey(), key) == 0) {
                return true;
            }
            return false;
        } catch (ClassCastException e) {
            return false;
        } catch (NoSuchElementException e2) {
            return false;
        } catch (NullPointerException e3) {
            return false;
        }
    }

    /* access modifiers changed from: protected */
    @Beta
    public V standardRemove(@Nullable Object key) {
        try {
            Iterator<Map.Entry<Object, V>> entryIterator = tailMap(key).entrySet().iterator();
            if (entryIterator.hasNext()) {
                Map.Entry<Object, V> ceilingEntry = entryIterator.next();
                if (unsafeCompare(ceilingEntry.getKey(), key) == 0) {
                    V value = ceilingEntry.getValue();
                    entryIterator.remove();
                    return value;
                }
            }
            return null;
        } catch (ClassCastException e) {
            return null;
        } catch (NullPointerException e2) {
            return null;
        }
    }

    /* access modifiers changed from: protected */
    @Beta
    public SortedMap<K, V> standardSubMap(K fromKey, K toKey) {
        return tailMap(fromKey).headMap(toKey);
    }
}
