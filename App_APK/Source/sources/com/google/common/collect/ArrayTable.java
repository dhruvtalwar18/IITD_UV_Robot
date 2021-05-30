package com.google.common.collect;

import com.google.common.annotations.Beta;
import com.google.common.base.Objects;
import com.google.common.base.Preconditions;
import com.google.common.collect.ImmutableMap;
import com.google.common.collect.Table;
import com.google.common.collect.Tables;
import java.io.Serializable;
import java.lang.reflect.Array;
import java.util.AbstractCollection;
import java.util.AbstractMap;
import java.util.AbstractSet;
import java.util.Arrays;
import java.util.Collection;
import java.util.Iterator;
import java.util.Map;
import java.util.Set;
import javax.annotation.Nullable;

@Beta
public final class ArrayTable<R, C, V> implements Table<R, C, V>, Serializable {
    private static final long serialVersionUID = 0;
    /* access modifiers changed from: private */
    public final V[][] array;
    private transient ArrayTable<R, C, V>.CellSet cellSet;
    /* access modifiers changed from: private */
    public final ImmutableMap<C, Integer> columnKeyToIndex;
    /* access modifiers changed from: private */
    public final ImmutableList<C> columnList;
    private transient ArrayTable<R, C, V>.ColumnMap columnMap;
    /* access modifiers changed from: private */
    public final ImmutableMap<R, Integer> rowKeyToIndex;
    /* access modifiers changed from: private */
    public final ImmutableList<R> rowList;
    private transient ArrayTable<R, C, V>.RowMap rowMap;
    private transient Collection<V> values;

    public static <R, C, V> ArrayTable<R, C, V> create(Iterable<? extends R> rowKeys, Iterable<? extends C> columnKeys) {
        return new ArrayTable<>(rowKeys, columnKeys);
    }

    public static <R, C, V> ArrayTable<R, C, V> create(Table<R, C, V> table) {
        return new ArrayTable<>(table);
    }

    public static <R, C, V> ArrayTable<R, C, V> create(ArrayTable<R, C, V> table) {
        return new ArrayTable<>(table);
    }

    private ArrayTable(Iterable<? extends R> rowKeys, Iterable<? extends C> columnKeys) {
        this.rowList = ImmutableList.copyOf(rowKeys);
        this.columnList = ImmutableList.copyOf(columnKeys);
        Preconditions.checkArgument(!this.rowList.isEmpty());
        Preconditions.checkArgument(!this.columnList.isEmpty());
        ImmutableMap.Builder<R, Integer> rowBuilder = ImmutableMap.builder();
        for (int i = 0; i < this.rowList.size(); i++) {
            rowBuilder.put(this.rowList.get(i), Integer.valueOf(i));
        }
        this.rowKeyToIndex = rowBuilder.build();
        ImmutableMap.Builder<C, Integer> columnBuilder = ImmutableMap.builder();
        for (int i2 = 0; i2 < this.columnList.size(); i2++) {
            columnBuilder.put(this.columnList.get(i2), Integer.valueOf(i2));
        }
        this.columnKeyToIndex = columnBuilder.build();
        this.array = (Object[][]) Array.newInstance(Object.class, new int[]{this.rowList.size(), this.columnList.size()});
    }

    private ArrayTable(Table<R, C, V> table) {
        this(table.rowKeySet(), table.columnKeySet());
        putAll(table);
    }

    private ArrayTable(ArrayTable<R, C, V> table) {
        this.rowList = table.rowList;
        this.columnList = table.columnList;
        this.rowKeyToIndex = table.rowKeyToIndex;
        this.columnKeyToIndex = table.columnKeyToIndex;
        V[][] copy = (Object[][]) Array.newInstance(Object.class, new int[]{this.rowList.size(), this.columnList.size()});
        this.array = copy;
        for (int i = 0; i < this.rowList.size(); i++) {
            System.arraycopy(table.array[i], 0, copy[i], 0, table.array[i].length);
        }
    }

    public ImmutableList<R> rowKeyList() {
        return this.rowList;
    }

    public ImmutableList<C> columnKeyList() {
        return this.columnList;
    }

    public V at(int rowIndex, int columnIndex) {
        return this.array[rowIndex][columnIndex];
    }

    public V set(int rowIndex, int columnIndex, @Nullable V value) {
        V oldValue = this.array[rowIndex][columnIndex];
        this.array[rowIndex][columnIndex] = value;
        return oldValue;
    }

    public V[][] toArray(Class<V> valueClass) {
        V[][] copy = (Object[][]) Array.newInstance(valueClass, new int[]{this.rowList.size(), this.columnList.size()});
        for (int i = 0; i < this.rowList.size(); i++) {
            System.arraycopy(this.array[i], 0, copy[i], 0, this.array[i].length);
        }
        return copy;
    }

    @Deprecated
    public void clear() {
        throw new UnsupportedOperationException();
    }

    public void eraseAll() {
        for (V[] row : this.array) {
            Arrays.fill(row, (Object) null);
        }
    }

    public boolean contains(@Nullable Object rowKey, @Nullable Object columnKey) {
        return containsRow(rowKey) && containsColumn(columnKey);
    }

    public boolean containsColumn(@Nullable Object columnKey) {
        return this.columnKeyToIndex.containsKey(columnKey);
    }

    public boolean containsRow(@Nullable Object rowKey) {
        return this.rowKeyToIndex.containsKey(rowKey);
    }

    public boolean containsValue(@Nullable Object value) {
        for (V[] arr$ : this.array) {
            for (V element : arr$[i$]) {
                if (Objects.equal(value, element)) {
                    return true;
                }
            }
        }
        return false;
    }

    public V get(@Nullable Object rowKey, @Nullable Object columnKey) {
        return getIndexed(this.rowKeyToIndex.get(rowKey), this.columnKeyToIndex.get(columnKey));
    }

    /* access modifiers changed from: private */
    public V getIndexed(Integer rowIndex, Integer columnIndex) {
        if (rowIndex == null || columnIndex == null) {
            return null;
        }
        return this.array[rowIndex.intValue()][columnIndex.intValue()];
    }

    public boolean isEmpty() {
        return false;
    }

    public V put(R rowKey, C columnKey, @Nullable V value) {
        Preconditions.checkNotNull(rowKey);
        Preconditions.checkNotNull(columnKey);
        Integer rowIndex = this.rowKeyToIndex.get(rowKey);
        Preconditions.checkArgument(rowIndex != null, "Row %s not in %s", rowKey, this.rowList);
        Integer columnIndex = this.columnKeyToIndex.get(columnKey);
        Preconditions.checkArgument(columnIndex != null, "Column %s not in %s", columnKey, this.columnList);
        return set(rowIndex.intValue(), columnIndex.intValue(), value);
    }

    public void putAll(Table<? extends R, ? extends C, ? extends V> table) {
        for (Table.Cell<? extends R, ? extends C, ? extends V> cell : table.cellSet()) {
            put(cell.getRowKey(), cell.getColumnKey(), cell.getValue());
        }
    }

    @Deprecated
    public V remove(Object rowKey, Object columnKey) {
        throw new UnsupportedOperationException();
    }

    public V erase(@Nullable Object rowKey, @Nullable Object columnKey) {
        Integer rowIndex = this.rowKeyToIndex.get(rowKey);
        Integer columnIndex = this.columnKeyToIndex.get(columnKey);
        if (rowIndex == null || columnIndex == null) {
            return null;
        }
        return set(rowIndex.intValue(), columnIndex.intValue(), (Object) null);
    }

    public int size() {
        return this.rowList.size() * this.columnList.size();
    }

    public boolean equals(@Nullable Object obj) {
        if (obj instanceof Table) {
            return cellSet().equals(((Table) obj).cellSet());
        }
        return false;
    }

    public int hashCode() {
        return cellSet().hashCode();
    }

    public String toString() {
        return rowMap().toString();
    }

    public Set<Table.Cell<R, C, V>> cellSet() {
        ArrayTable<R, C, V>.CellSet set = this.cellSet;
        if (set != null) {
            return set;
        }
        ArrayTable<R, C, V>.CellSet cellSet2 = new CellSet();
        this.cellSet = cellSet2;
        return cellSet2;
    }

    private class CellSet extends AbstractSet<Table.Cell<R, C, V>> {
        private CellSet() {
        }

        public Iterator<Table.Cell<R, C, V>> iterator() {
            return new AbstractIndexedListIterator<Table.Cell<R, C, V>>(size()) {
                /* access modifiers changed from: protected */
                public Table.Cell<R, C, V> get(final int index) {
                    return new Tables.AbstractCell<R, C, V>() {
                        final int columnIndex = (index % ArrayTable.this.columnList.size());
                        final int rowIndex = (index / ArrayTable.this.columnList.size());

                        public R getRowKey() {
                            return ArrayTable.this.rowList.get(this.rowIndex);
                        }

                        public C getColumnKey() {
                            return ArrayTable.this.columnList.get(this.columnIndex);
                        }

                        public V getValue() {
                            return ArrayTable.this.array[this.rowIndex][this.columnIndex];
                        }
                    };
                }
            };
        }

        public int size() {
            return ArrayTable.this.size();
        }

        public boolean contains(Object obj) {
            if (!(obj instanceof Table.Cell)) {
                return false;
            }
            Table.Cell<?, ?, ?> cell = (Table.Cell) obj;
            Integer rowIndex = (Integer) ArrayTable.this.rowKeyToIndex.get(cell.getRowKey());
            Integer columnIndex = (Integer) ArrayTable.this.columnKeyToIndex.get(cell.getColumnKey());
            if (rowIndex == null || columnIndex == null || !Objects.equal(ArrayTable.this.array[rowIndex.intValue()][columnIndex.intValue()], cell.getValue())) {
                return false;
            }
            return true;
        }
    }

    public Map<R, V> column(C columnKey) {
        Preconditions.checkNotNull(columnKey);
        Integer columnIndex = this.columnKeyToIndex.get(columnKey);
        return columnIndex == null ? ImmutableMap.of() : new Column(columnIndex.intValue());
    }

    private class Column extends AbstractMap<R, V> {
        final int columnIndex;
        ArrayTable<R, C, V>.ColumnEntrySet entrySet;

        Column(int columnIndex2) {
            this.columnIndex = columnIndex2;
        }

        public Set<Map.Entry<R, V>> entrySet() {
            ArrayTable<R, C, V>.ColumnEntrySet set = this.entrySet;
            if (set != null) {
                return set;
            }
            ArrayTable<R, C, V>.ColumnEntrySet columnEntrySet = new ColumnEntrySet(this.columnIndex);
            this.entrySet = columnEntrySet;
            return columnEntrySet;
        }

        public V get(Object rowKey) {
            return ArrayTable.this.getIndexed((Integer) ArrayTable.this.rowKeyToIndex.get(rowKey), Integer.valueOf(this.columnIndex));
        }

        public boolean containsKey(Object rowKey) {
            return ArrayTable.this.rowKeyToIndex.containsKey(rowKey);
        }

        public V put(R rowKey, V value) {
            Preconditions.checkNotNull(rowKey);
            Integer rowIndex = (Integer) ArrayTable.this.rowKeyToIndex.get(rowKey);
            Preconditions.checkArgument(rowIndex != null, "Row %s not in %s", rowKey, ArrayTable.this.rowList);
            return ArrayTable.this.set(rowIndex.intValue(), this.columnIndex, value);
        }

        public Set<R> keySet() {
            return ArrayTable.this.rowKeySet();
        }
    }

    private class ColumnEntrySet extends AbstractSet<Map.Entry<R, V>> {
        final int columnIndex;

        ColumnEntrySet(int columnIndex2) {
            this.columnIndex = columnIndex2;
        }

        public Iterator<Map.Entry<R, V>> iterator() {
            return new AbstractIndexedListIterator<Map.Entry<R, V>>(size()) {
                /* access modifiers changed from: protected */
                public Map.Entry<R, V> get(final int rowIndex) {
                    return new AbstractMapEntry<R, V>() {
                        public R getKey() {
                            return ArrayTable.this.rowList.get(rowIndex);
                        }

                        public V getValue() {
                            return ArrayTable.this.array[rowIndex][ColumnEntrySet.this.columnIndex];
                        }

                        public V setValue(V value) {
                            return ArrayTable.this.set(rowIndex, ColumnEntrySet.this.columnIndex, value);
                        }
                    };
                }
            };
        }

        public int size() {
            return ArrayTable.this.rowList.size();
        }
    }

    public ImmutableSet<C> columnKeySet() {
        return this.columnKeyToIndex.keySet();
    }

    public Map<C, Map<R, V>> columnMap() {
        ArrayTable<R, C, V>.ColumnMap map = this.columnMap;
        if (map != null) {
            return map;
        }
        ArrayTable<R, C, V>.ColumnMap columnMap2 = new ColumnMap();
        this.columnMap = columnMap2;
        return columnMap2;
    }

    private class ColumnMap extends AbstractMap<C, Map<R, V>> {
        transient ArrayTable<R, C, V>.ColumnMapEntrySet entrySet;

        private ColumnMap() {
        }

        public Set<Map.Entry<C, Map<R, V>>> entrySet() {
            ArrayTable<R, C, V>.ColumnMapEntrySet set = this.entrySet;
            if (set != null) {
                return set;
            }
            ArrayTable<R, C, V>.ColumnMapEntrySet columnMapEntrySet = new ColumnMapEntrySet();
            this.entrySet = columnMapEntrySet;
            return columnMapEntrySet;
        }

        public Map<R, V> get(Object columnKey) {
            Integer columnIndex = (Integer) ArrayTable.this.columnKeyToIndex.get(columnKey);
            if (columnIndex == null) {
                return null;
            }
            return new Column(columnIndex.intValue());
        }

        public boolean containsKey(Object columnKey) {
            return ArrayTable.this.containsColumn(columnKey);
        }

        public Set<C> keySet() {
            return ArrayTable.this.columnKeySet();
        }

        public Map<R, V> remove(Object columnKey) {
            throw new UnsupportedOperationException();
        }
    }

    private class ColumnMapEntrySet extends AbstractSet<Map.Entry<C, Map<R, V>>> {
        private ColumnMapEntrySet() {
        }

        public Iterator<Map.Entry<C, Map<R, V>>> iterator() {
            return new AbstractIndexedListIterator<Map.Entry<C, Map<R, V>>>(size()) {
                /* access modifiers changed from: protected */
                public Map.Entry<C, Map<R, V>> get(int index) {
                    return Maps.immutableEntry(ArrayTable.this.columnList.get(index), new Column(index));
                }
            };
        }

        public int size() {
            return ArrayTable.this.columnList.size();
        }
    }

    public Map<C, V> row(R rowKey) {
        Preconditions.checkNotNull(rowKey);
        Integer rowIndex = this.rowKeyToIndex.get(rowKey);
        return rowIndex == null ? ImmutableMap.of() : new Row(rowIndex.intValue());
    }

    private class Row extends AbstractMap<C, V> {
        ArrayTable<R, C, V>.RowEntrySet entrySet;
        final int rowIndex;

        Row(int rowIndex2) {
            this.rowIndex = rowIndex2;
        }

        public Set<Map.Entry<C, V>> entrySet() {
            ArrayTable<R, C, V>.RowEntrySet set = this.entrySet;
            if (set != null) {
                return set;
            }
            ArrayTable<R, C, V>.RowEntrySet rowEntrySet = new RowEntrySet(this.rowIndex);
            this.entrySet = rowEntrySet;
            return rowEntrySet;
        }

        public V get(Object columnKey) {
            return ArrayTable.this.getIndexed(Integer.valueOf(this.rowIndex), (Integer) ArrayTable.this.columnKeyToIndex.get(columnKey));
        }

        public boolean containsKey(Object columnKey) {
            return ArrayTable.this.containsColumn(columnKey);
        }

        public V put(C columnKey, V value) {
            Preconditions.checkNotNull(columnKey);
            Integer columnIndex = (Integer) ArrayTable.this.columnKeyToIndex.get(columnKey);
            Preconditions.checkArgument(columnIndex != null, "Column %s not in %s", columnKey, ArrayTable.this.columnList);
            return ArrayTable.this.set(this.rowIndex, columnIndex.intValue(), value);
        }

        public Set<C> keySet() {
            return ArrayTable.this.columnKeySet();
        }
    }

    private class RowEntrySet extends AbstractSet<Map.Entry<C, V>> {
        final int rowIndex;

        RowEntrySet(int rowIndex2) {
            this.rowIndex = rowIndex2;
        }

        public Iterator<Map.Entry<C, V>> iterator() {
            return new AbstractIndexedListIterator<Map.Entry<C, V>>(size()) {
                /* access modifiers changed from: protected */
                public Map.Entry<C, V> get(final int columnIndex) {
                    return new AbstractMapEntry<C, V>() {
                        public C getKey() {
                            return ArrayTable.this.columnList.get(columnIndex);
                        }

                        public V getValue() {
                            return ArrayTable.this.array[RowEntrySet.this.rowIndex][columnIndex];
                        }

                        public V setValue(V value) {
                            return ArrayTable.this.set(RowEntrySet.this.rowIndex, columnIndex, value);
                        }
                    };
                }
            };
        }

        public int size() {
            return ArrayTable.this.columnList.size();
        }
    }

    public ImmutableSet<R> rowKeySet() {
        return this.rowKeyToIndex.keySet();
    }

    public Map<R, Map<C, V>> rowMap() {
        ArrayTable<R, C, V>.RowMap map = this.rowMap;
        if (map != null) {
            return map;
        }
        ArrayTable<R, C, V>.RowMap rowMap2 = new RowMap();
        this.rowMap = rowMap2;
        return rowMap2;
    }

    private class RowMap extends AbstractMap<R, Map<C, V>> {
        transient ArrayTable<R, C, V>.RowMapEntrySet entrySet;

        private RowMap() {
        }

        public Set<Map.Entry<R, Map<C, V>>> entrySet() {
            ArrayTable<R, C, V>.RowMapEntrySet set = this.entrySet;
            if (set != null) {
                return set;
            }
            ArrayTable<R, C, V>.RowMapEntrySet rowMapEntrySet = new RowMapEntrySet();
            this.entrySet = rowMapEntrySet;
            return rowMapEntrySet;
        }

        public Map<C, V> get(Object rowKey) {
            Integer rowIndex = (Integer) ArrayTable.this.rowKeyToIndex.get(rowKey);
            if (rowIndex == null) {
                return null;
            }
            return new Row(rowIndex.intValue());
        }

        public boolean containsKey(Object rowKey) {
            return ArrayTable.this.containsRow(rowKey);
        }

        public Set<R> keySet() {
            return ArrayTable.this.rowKeySet();
        }

        public Map<C, V> remove(Object rowKey) {
            throw new UnsupportedOperationException();
        }
    }

    private class RowMapEntrySet extends AbstractSet<Map.Entry<R, Map<C, V>>> {
        private RowMapEntrySet() {
        }

        public Iterator<Map.Entry<R, Map<C, V>>> iterator() {
            return new AbstractIndexedListIterator<Map.Entry<R, Map<C, V>>>(size()) {
                /* access modifiers changed from: protected */
                public Map.Entry<R, Map<C, V>> get(int index) {
                    return Maps.immutableEntry(ArrayTable.this.rowList.get(index), new Row(index));
                }
            };
        }

        public int size() {
            return ArrayTable.this.rowList.size();
        }
    }

    public Collection<V> values() {
        Collection<V> v = this.values;
        if (v != null) {
            return v;
        }
        Values values2 = new Values();
        this.values = values2;
        return values2;
    }

    private class Values extends AbstractCollection<V> {
        private Values() {
        }

        public Iterator<V> iterator() {
            return new AbstractIndexedListIterator<V>(size()) {
                /* access modifiers changed from: protected */
                public V get(int index) {
                    return ArrayTable.this.array[index / ArrayTable.this.columnList.size()][index % ArrayTable.this.columnList.size()];
                }
            };
        }

        public int size() {
            return ArrayTable.this.size();
        }

        public boolean contains(Object value) {
            return ArrayTable.this.containsValue(value);
        }
    }
}
