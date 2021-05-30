package com.google.common.collect;

import com.google.common.annotations.GwtCompatible;
import com.google.common.base.Preconditions;
import com.google.common.base.Supplier;
import java.io.Serializable;
import java.util.Collection;
import java.util.HashMap;
import java.util.Map;
import java.util.Set;
import javax.annotation.Nullable;

@GwtCompatible(serializable = true)
public class HashBasedTable<R, C, V> extends StandardTable<R, C, V> {
    private static final long serialVersionUID = 0;

    public /* bridge */ /* synthetic */ Set cellSet() {
        return super.cellSet();
    }

    public /* bridge */ /* synthetic */ void clear() {
        super.clear();
    }

    public /* bridge */ /* synthetic */ Map column(Object x0) {
        return super.column(x0);
    }

    public /* bridge */ /* synthetic */ Set columnKeySet() {
        return super.columnKeySet();
    }

    public /* bridge */ /* synthetic */ Map columnMap() {
        return super.columnMap();
    }

    public /* bridge */ /* synthetic */ int hashCode() {
        return super.hashCode();
    }

    public /* bridge */ /* synthetic */ boolean isEmpty() {
        return super.isEmpty();
    }

    public /* bridge */ /* synthetic */ Object put(Object x0, Object x1, Object x2) {
        return super.put(x0, x1, x2);
    }

    public /* bridge */ /* synthetic */ void putAll(Table x0) {
        super.putAll(x0);
    }

    public /* bridge */ /* synthetic */ Map row(Object x0) {
        return super.row(x0);
    }

    public /* bridge */ /* synthetic */ Set rowKeySet() {
        return super.rowKeySet();
    }

    public /* bridge */ /* synthetic */ Map rowMap() {
        return super.rowMap();
    }

    public /* bridge */ /* synthetic */ int size() {
        return super.size();
    }

    public /* bridge */ /* synthetic */ String toString() {
        return super.toString();
    }

    public /* bridge */ /* synthetic */ Collection values() {
        return super.values();
    }

    private static class Factory<C, V> implements Supplier<Map<C, V>>, Serializable {
        private static final long serialVersionUID = 0;
        final int expectedSize;

        Factory(int expectedSize2) {
            this.expectedSize = expectedSize2;
        }

        public Map<C, V> get() {
            return Maps.newHashMapWithExpectedSize(this.expectedSize);
        }
    }

    public static <R, C, V> HashBasedTable<R, C, V> create() {
        return new HashBasedTable<>(new HashMap(), new Factory(0));
    }

    public static <R, C, V> HashBasedTable<R, C, V> create(int expectedRows, int expectedCellsPerRow) {
        Preconditions.checkArgument(expectedCellsPerRow >= 0);
        return new HashBasedTable<>(Maps.newHashMapWithExpectedSize(expectedRows), new Factory(expectedCellsPerRow));
    }

    public static <R, C, V> HashBasedTable<R, C, V> create(Table<? extends R, ? extends C, ? extends V> table) {
        HashBasedTable<R, C, V> result = create();
        result.putAll(table);
        return result;
    }

    HashBasedTable(Map<R, Map<C, V>> backingMap, Factory<C, V> factory) {
        super(backingMap, factory);
    }

    public boolean contains(@Nullable Object rowKey, @Nullable Object columnKey) {
        return super.contains(rowKey, columnKey);
    }

    public boolean containsColumn(@Nullable Object columnKey) {
        return super.containsColumn(columnKey);
    }

    public boolean containsRow(@Nullable Object rowKey) {
        return super.containsRow(rowKey);
    }

    public boolean containsValue(@Nullable Object value) {
        return super.containsValue(value);
    }

    public V get(@Nullable Object rowKey, @Nullable Object columnKey) {
        return super.get(rowKey, columnKey);
    }

    public boolean equals(@Nullable Object obj) {
        return super.equals(obj);
    }

    public V remove(@Nullable Object rowKey, @Nullable Object columnKey) {
        return super.remove(rowKey, columnKey);
    }
}
