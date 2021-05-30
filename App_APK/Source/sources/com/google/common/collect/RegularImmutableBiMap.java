package com.google.common.collect;

import com.google.common.annotations.GwtCompatible;
import com.google.common.collect.ImmutableMap;
import java.util.Iterator;
import java.util.Map;

@GwtCompatible(emulated = true, serializable = true)
class RegularImmutableBiMap<K, V> extends ImmutableBiMap<K, V> {
    final transient ImmutableMap<K, V> delegate;
    final transient ImmutableBiMap<V, K> inverse;

    RegularImmutableBiMap(ImmutableMap<K, V> delegate2) {
        this.delegate = delegate2;
        ImmutableMap.Builder<V, K> builder = ImmutableMap.builder();
        Iterator i$ = delegate2.entrySet().iterator();
        while (i$.hasNext()) {
            Map.Entry<K, V> entry = i$.next();
            builder.put(entry.getValue(), entry.getKey());
        }
        this.inverse = new RegularImmutableBiMap(builder.build(), this);
    }

    RegularImmutableBiMap(ImmutableMap<K, V> delegate2, ImmutableBiMap<V, K> inverse2) {
        this.delegate = delegate2;
        this.inverse = inverse2;
    }

    /* access modifiers changed from: package-private */
    public ImmutableMap<K, V> delegate() {
        return this.delegate;
    }

    public ImmutableBiMap<V, K> inverse() {
        return this.inverse;
    }

    /* access modifiers changed from: package-private */
    public boolean isPartialView() {
        return this.delegate.isPartialView() || this.inverse.delegate().isPartialView();
    }
}
