package com.google.common.collect;

import com.google.common.annotations.GwtCompatible;
import com.google.common.base.Objects;
import com.google.common.base.Preconditions;
import com.google.common.collect.Table;
import java.util.Map;
import javax.annotation.Nullable;

@GwtCompatible
final class SingletonImmutableTable<R, C, V> extends ImmutableTable<R, C, V> {
    private final C singleColumnKey;
    private final R singleRowKey;
    private final V singleValue;

    SingletonImmutableTable(R rowKey, C columnKey, V value) {
        this.singleRowKey = Preconditions.checkNotNull(rowKey);
        this.singleColumnKey = Preconditions.checkNotNull(columnKey);
        this.singleValue = Preconditions.checkNotNull(value);
    }

    SingletonImmutableTable(Table.Cell<R, C, V> cell) {
        this(cell.getRowKey(), cell.getColumnKey(), cell.getValue());
    }

    public ImmutableSet<Table.Cell<R, C, V>> cellSet() {
        return ImmutableSet.of(Tables.immutableCell(this.singleRowKey, this.singleColumnKey, this.singleValue));
    }

    public ImmutableMap<R, V> column(C columnKey) {
        Preconditions.checkNotNull(columnKey);
        return containsColumn(columnKey) ? ImmutableMap.of(this.singleRowKey, this.singleValue) : ImmutableMap.of();
    }

    public ImmutableSet<C> columnKeySet() {
        return ImmutableSet.of(this.singleColumnKey);
    }

    public ImmutableMap<C, Map<R, V>> columnMap() {
        return ImmutableMap.of(this.singleColumnKey, ImmutableMap.of(this.singleRowKey, this.singleValue));
    }

    public boolean contains(@Nullable Object rowKey, @Nullable Object columnKey) {
        return containsRow(rowKey) && containsColumn(columnKey);
    }

    public boolean containsColumn(@Nullable Object columnKey) {
        return Objects.equal(this.singleColumnKey, columnKey);
    }

    public boolean containsRow(@Nullable Object rowKey) {
        return Objects.equal(this.singleRowKey, rowKey);
    }

    public boolean containsValue(@Nullable Object value) {
        return Objects.equal(this.singleValue, value);
    }

    public V get(@Nullable Object rowKey, @Nullable Object columnKey) {
        if (contains(rowKey, columnKey)) {
            return this.singleValue;
        }
        return null;
    }

    public boolean isEmpty() {
        return false;
    }

    public ImmutableMap<C, V> row(R rowKey) {
        Preconditions.checkNotNull(rowKey);
        return containsRow(rowKey) ? ImmutableMap.of(this.singleColumnKey, this.singleValue) : ImmutableMap.of();
    }

    public ImmutableSet<R> rowKeySet() {
        return ImmutableSet.of(this.singleRowKey);
    }

    public ImmutableMap<R, Map<C, V>> rowMap() {
        return ImmutableMap.of(this.singleRowKey, ImmutableMap.of(this.singleColumnKey, this.singleValue));
    }

    public int size() {
        return 1;
    }

    public ImmutableCollection<V> values() {
        return ImmutableSet.of(this.singleValue);
    }

    public boolean equals(@Nullable Object obj) {
        if (obj == this) {
            return true;
        }
        if (obj instanceof Table) {
            Table<?, ?, ?> that = (Table) obj;
            if (that.size() == 1) {
                Table.Cell<?, ?, ?> thatCell = that.cellSet().iterator().next();
                if (!Objects.equal(this.singleRowKey, thatCell.getRowKey()) || !Objects.equal(this.singleColumnKey, thatCell.getColumnKey()) || !Objects.equal(this.singleValue, thatCell.getValue())) {
                    return false;
                }
                return true;
            }
        }
        return false;
    }

    public int hashCode() {
        return Objects.hashCode(this.singleRowKey, this.singleColumnKey, this.singleValue);
    }

    public String toString() {
        return '{' + this.singleRowKey + "={" + this.singleColumnKey + '=' + this.singleValue + "}}";
    }
}
