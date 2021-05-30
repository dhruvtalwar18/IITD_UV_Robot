package com.google.common.cache;

import com.google.common.annotations.Beta;

@Beta
public interface Weigher<K, V> {
    int weigh(K k, V v);
}
