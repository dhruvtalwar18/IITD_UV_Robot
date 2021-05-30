package com.google.common.cache;

import com.google.common.annotations.Beta;

@Beta
public interface RemovalListener<K, V> {
    void onRemoval(RemovalNotification<K, V> removalNotification);
}
