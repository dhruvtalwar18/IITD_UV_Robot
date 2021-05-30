package com.google.common.collect;

import com.google.common.annotations.Beta;
import com.google.common.annotations.GwtCompatible;
import com.google.common.base.Function;
import com.google.common.base.Objects;
import com.google.common.base.Preconditions;
import com.google.common.base.Supplier;
import com.google.common.collect.Collections2;
import com.google.common.collect.Table;
import java.io.Serializable;
import java.util.Collection;
import java.util.Collections;
import java.util.Map;
import java.util.Set;
import java.util.SortedMap;
import java.util.SortedSet;
import javax.annotation.Nullable;

@GwtCompatible
@Beta
public final class Tables {
    private static final Function<? extends Map<?, ?>, ? extends Map<?, ?>> UNMODIFIABLE_WRAPPER = new Function<Map<Object, Object>, Map<Object, Object>>() {
        public Map<Object, Object> apply(Map<Object, Object> input) {
            return Collections.unmodifiableMap(input);
        }
    };

    private Tables() {
    }

    public static <R, C, V> Table.Cell<R, C, V> immutableCell(@Nullable R rowKey, @Nullable C columnKey, @Nullable V value) {
        return new ImmutableCell(rowKey, columnKey, value);
    }

    static final class ImmutableCell<R, C, V> extends AbstractCell<R, C, V> implements Serializable {
        private static final long serialVersionUID = 0;
        private final C columnKey;
        private final R rowKey;
        private final V value;

        ImmutableCell(@Nullable R rowKey2, @Nullable C columnKey2, @Nullable V value2) {
            this.rowKey = rowKey2;
            this.columnKey = columnKey2;
            this.value = value2;
        }

        public R getRowKey() {
            return this.rowKey;
        }

        public C getColumnKey() {
            return this.columnKey;
        }

        public V getValue() {
            return this.value;
        }
    }

    static abstract class AbstractCell<R, C, V> implements Table.Cell<R, C, V> {
        AbstractCell() {
        }

        public boolean equals(Object obj) {
            if (obj == this) {
                return true;
            }
            if (!(obj instanceof Table.Cell)) {
                return false;
            }
            Table.Cell<?, ?, ?> other = (Table.Cell) obj;
            if (!Objects.equal(getRowKey(), other.getRowKey()) || !Objects.equal(getColumnKey(), other.getColumnKey()) || !Objects.equal(getValue(), other.getValue())) {
                return false;
            }
            return true;
        }

        public int hashCode() {
            return Objects.hashCode(getRowKey(), getColumnKey(), getValue());
        }

        public String toString() {
            return "(" + getRowKey() + "," + getColumnKey() + ")=" + getValue();
        }
    }

    public static <R, C, V> Table<C, R, V> transpose(Table<R, C, V> table) {
        return table instanceof TransposeTable ? ((TransposeTable) table).original : new TransposeTable(table);
    }

    private static class TransposeTable<C, R, V> implements Table<C, R, V> {
        /* access modifiers changed from: private */
        public static final Function<Table.Cell<?, ?, ?>, Table.Cell<?, ?, ?>> TRANSPOSE_CELL = new Function<Table.Cell<?, ?, ?>, Table.Cell<?, ?, ?>>() {
            public Table.Cell<?, ?, ?> apply(Table.Cell<?, ?, ?> cell) {
                return Tables.immutableCell(cell.getColumnKey(), cell.getRowKey(), cell.getValue());
            }
        };
        TransposeTable<C, R, V>.CellSet cellSet;
        final Table<R, C, V> original;

        TransposeTable(Table<R, C, V> original2) {
            this.original = (Table) Preconditions.checkNotNull(original2);
        }

        public void clear() {
            this.original.clear();
        }

        public Map<C, V> column(R columnKey) {
            return this.original.row(columnKey);
        }

        public Set<R> columnKeySet() {
            return this.original.rowKeySet();
        }

        public Map<R, Map<C, V>> columnMap() {
            return this.original.rowMap();
        }

        public boolean contains(@Nullable Object rowKey, @Nullable Object columnKey) {
            return this.original.contains(columnKey, rowKey);
        }

        public boolean containsColumn(@Nullable Object columnKey) {
            return this.original.containsRow(columnKey);
        }

        public boolean containsRow(@Nullable Object rowKey) {
            return this.original.containsColumn(rowKey);
        }

        public boolean containsValue(@Nullable Object value) {
            return this.original.containsValue(value);
        }

        public V get(@Nullable Object rowKey, @Nullable Object columnKey) {
            return this.original.get(columnKey, rowKey);
        }

        public boolean isEmpty() {
            return this.original.isEmpty();
        }

        public V put(C rowKey, R columnKey, V value) {
            return this.original.put(columnKey, rowKey, value);
        }

        public void putAll(Table<? extends C, ? extends R, ? extends V> table) {
            this.original.putAll(Tables.transpose(table));
        }

        public V remove(@Nullable Object rowKey, @Nullable Object columnKey) {
            return this.original.remove(columnKey, rowKey);
        }

        public Map<R, V> row(C rowKey) {
            return this.original.column(rowKey);
        }

        public Set<C> rowKeySet() {
            return this.original.columnKeySet();
        }

        public Map<C, Map<R, V>> rowMap() {
            return this.original.columnMap();
        }

        public int size() {
            return this.original.size();
        }

        public Collection<V> values() {
            return this.original.values();
        }

        public boolean equals(@Nullable Object obj) {
            if (obj == this) {
                return true;
            }
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

        public Set<Table.Cell<C, R, V>> cellSet() {
            TransposeTable<C, R, V>.CellSet result = this.cellSet;
            if (result != null) {
                return result;
            }
            TransposeTable<C, R, V>.CellSet cellSet2 = new CellSet();
            this.cellSet = cellSet2;
            return cellSet2;
        }

        class CellSet extends Collections2.TransformedCollection<Table.Cell<R, C, V>, Table.Cell<C, R, V>> implements Set<Table.Cell<C, R, V>> {
            CellSet() {
                super(TransposeTable.this.original.cellSet(), TransposeTable.TRANSPOSE_CELL);
            }

            public boolean equals(Object obj) {
                if (obj == this) {
                    return true;
                }
                if (!(obj instanceof Set)) {
                    return false;
                }
                Set<?> os = (Set) obj;
                if (os.size() != size()) {
                    return false;
                }
                return containsAll(os);
            }

            public int hashCode() {
                return Sets.hashCodeImpl(this);
            }

            public boolean contains(Object obj) {
                if (!(obj instanceof Table.Cell)) {
                    return false;
                }
                Table.Cell<?, ?, ?> cell = (Table.Cell) obj;
                return TransposeTable.this.original.cellSet().contains(Tables.immutableCell(cell.getColumnKey(), cell.getRowKey(), cell.getValue()));
            }

            public boolean remove(Object obj) {
                if (!(obj instanceof Table.Cell)) {
                    return false;
                }
                Table.Cell<?, ?, ?> cell = (Table.Cell) obj;
                return TransposeTable.this.original.cellSet().remove(Tables.immutableCell(cell.getColumnKey(), cell.getRowKey(), cell.getValue()));
            }
        }
    }

    public static <R, C, V> Table<R, C, V> newCustomTable(Map<R, Map<C, V>> backingMap, Supplier<? extends Map<C, V>> factory) {
        Preconditions.checkArgument(backingMap.isEmpty());
        Preconditions.checkNotNull(factory);
        return new StandardTable(backingMap, factory);
    }

    public static <R, C, V1, V2> Table<R, C, V2> transformValues(Table<R, C, V1> fromTable, Function<? super V1, V2> function) {
        return new TransformedTable(fromTable, function);
    }

    private static class TransformedTable<R, C, V1, V2> implements Table<R, C, V2> {
        TransformedTable<R, C, V1, V2>.CellSet cellSet;
        Map<C, Map<R, V2>> columnMap;
        final Table<R, C, V1> fromTable;
        final Function<? super V1, V2> function;
        Map<R, Map<C, V2>> rowMap;
        Collection<V2> values;

        TransformedTable(Table<R, C, V1> fromTable2, Function<? super V1, V2> function2) {
            this.fromTable = (Table) Preconditions.checkNotNull(fromTable2);
            this.function = (Function) Preconditions.checkNotNull(function2);
        }

        public boolean contains(Object rowKey, Object columnKey) {
            return this.fromTable.contains(rowKey, columnKey);
        }

        public boolean containsRow(Object rowKey) {
            return this.fromTable.containsRow(rowKey);
        }

        public boolean containsColumn(Object columnKey) {
            return this.fromTable.containsColumn(columnKey);
        }

        public boolean containsValue(Object value) {
            return values().contains(value);
        }

        public V2 get(Object rowKey, Object columnKey) {
            if (contains(rowKey, columnKey)) {
                return this.function.apply(this.fromTable.get(rowKey, columnKey));
            }
            return null;
        }

        public boolean isEmpty() {
            return this.fromTable.isEmpty();
        }

        public int size() {
            return this.fromTable.size();
        }

        public void clear() {
            this.fromTable.clear();
        }

        public V2 put(R r, C c, V2 v2) {
            throw new UnsupportedOperationException();
        }

        public void putAll(Table<? extends R, ? extends C, ? extends V2> table) {
            throw new UnsupportedOperationException();
        }

        public V2 remove(Object rowKey, Object columnKey) {
            if (contains(rowKey, columnKey)) {
                return this.function.apply(this.fromTable.remove(rowKey, columnKey));
            }
            return null;
        }

        public Map<C, V2> row(R rowKey) {
            return Maps.transformValues(this.fromTable.row(rowKey), this.function);
        }

        public Map<R, V2> column(C columnKey) {
            return Maps.transformValues(this.fromTable.column(columnKey), this.function);
        }

        /* access modifiers changed from: package-private */
        public Function<Table.Cell<R, C, V1>, Table.Cell<R, C, V2>> cellFunction() {
            return new Function<Table.Cell<R, C, V1>, Table.Cell<R, C, V2>>() {
                public Table.Cell<R, C, V2> apply(Table.Cell<R, C, V1> cell) {
                    return Tables.immutableCell(cell.getRowKey(), cell.getColumnKey(), TransformedTable.this.function.apply(cell.getValue()));
                }
            };
        }

        class CellSet extends Collections2.TransformedCollection<Table.Cell<R, C, V1>, Table.Cell<R, C, V2>> implements Set<Table.Cell<R, C, V2>> {
            CellSet() {
                super(TransformedTable.this.fromTable.cellSet(), TransformedTable.this.cellFunction());
            }

            public boolean equals(Object obj) {
                return Sets.equalsImpl(this, obj);
            }

            public int hashCode() {
                return Sets.hashCodeImpl(this);
            }

            public boolean contains(Object obj) {
                if (!(obj instanceof Table.Cell)) {
                    return false;
                }
                Table.Cell<?, ?, ?> cell = (Table.Cell) obj;
                if (!Objects.equal(cell.getValue(), TransformedTable.this.get(cell.getRowKey(), cell.getColumnKey()))) {
                    return false;
                }
                if (cell.getValue() != null || TransformedTable.this.fromTable.contains(cell.getRowKey(), cell.getColumnKey())) {
                    return true;
                }
                return false;
            }

            public boolean remove(Object obj) {
                if (!contains(obj)) {
                    return false;
                }
                Table.Cell<?, ?, ?> cell = (Table.Cell) obj;
                TransformedTable.this.fromTable.remove(cell.getRowKey(), cell.getColumnKey());
                return true;
            }
        }

        public Set<Table.Cell<R, C, V2>> cellSet() {
            if (this.cellSet != null) {
                return this.cellSet;
            }
            TransformedTable<R, C, V1, V2>.CellSet cellSet2 = new CellSet();
            this.cellSet = cellSet2;
            return cellSet2;
        }

        public Set<R> rowKeySet() {
            return this.fromTable.rowKeySet();
        }

        public Set<C> columnKeySet() {
            return this.fromTable.columnKeySet();
        }

        public Collection<V2> values() {
            if (this.values != null) {
                return this.values;
            }
            Collection<V2> transform = Collections2.transform(this.fromTable.values(), this.function);
            this.values = transform;
            return transform;
        }

        /* access modifiers changed from: package-private */
        public Map<R, Map<C, V2>> createRowMap() {
            return Maps.transformValues(this.fromTable.rowMap(), new Function<Map<C, V1>, Map<C, V2>>() {
                public Map<C, V2> apply(Map<C, V1> row) {
                    return Maps.transformValues(row, TransformedTable.this.function);
                }
            });
        }

        public Map<R, Map<C, V2>> rowMap() {
            if (this.rowMap != null) {
                return this.rowMap;
            }
            Map<R, Map<C, V2>> createRowMap = createRowMap();
            this.rowMap = createRowMap;
            return createRowMap;
        }

        /* access modifiers changed from: package-private */
        public Map<C, Map<R, V2>> createColumnMap() {
            return Maps.transformValues(this.fromTable.columnMap(), new Function<Map<R, V1>, Map<R, V2>>() {
                public Map<R, V2> apply(Map<R, V1> column) {
                    return Maps.transformValues(column, TransformedTable.this.function);
                }
            });
        }

        public Map<C, Map<R, V2>> columnMap() {
            if (this.columnMap != null) {
                return this.columnMap;
            }
            Map<C, Map<R, V2>> createColumnMap = createColumnMap();
            this.columnMap = createColumnMap;
            return createColumnMap;
        }

        public boolean equals(@Nullable Object obj) {
            if (obj == this) {
                return true;
            }
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
    }

    public static <R, C, V> Table<R, C, V> unmodifiableTable(Table<? extends R, ? extends C, ? extends V> table) {
        return new UnmodifiableTable(table);
    }

    private static class UnmodifiableTable<R, C, V> extends ForwardingTable<R, C, V> implements Serializable {
        private static final long serialVersionUID = 0;
        final Table<? extends R, ? extends C, ? extends V> delegate;

        UnmodifiableTable(Table<? extends R, ? extends C, ? extends V> delegate2) {
            this.delegate = (Table) Preconditions.checkNotNull(delegate2);
        }

        /* access modifiers changed from: protected */
        public Table<R, C, V> delegate() {
            return this.delegate;
        }

        public Set<Table.Cell<R, C, V>> cellSet() {
            return Collections.unmodifiableSet(super.cellSet());
        }

        public void clear() {
            throw new UnsupportedOperationException();
        }

        public Map<R, V> column(@Nullable C columnKey) {
            return Collections.unmodifiableMap(super.column(columnKey));
        }

        public Set<C> columnKeySet() {
            return Collections.unmodifiableSet(super.columnKeySet());
        }

        public Map<C, Map<R, V>> columnMap() {
            return Collections.unmodifiableMap(Maps.transformValues(super.columnMap(), Tables.unmodifiableWrapper()));
        }

        public V put(@Nullable R r, @Nullable C c, @Nullable V v) {
            throw new UnsupportedOperationException();
        }

        public void putAll(Table<? extends R, ? extends C, ? extends V> table) {
            throw new UnsupportedOperationException();
        }

        public V remove(@Nullable Object rowKey, @Nullable Object columnKey) {
            throw new UnsupportedOperationException();
        }

        public Map<C, V> row(@Nullable R rowKey) {
            return Collections.unmodifiableMap(super.row(rowKey));
        }

        public Set<R> rowKeySet() {
            return Collections.unmodifiableSet(super.rowKeySet());
        }

        public Map<R, Map<C, V>> rowMap() {
            return Collections.unmodifiableMap(Maps.transformValues(super.rowMap(), Tables.unmodifiableWrapper()));
        }

        public Collection<V> values() {
            return Collections.unmodifiableCollection(super.values());
        }
    }

    public static <R, C, V> RowSortedTable<R, C, V> unmodifiableRowSortedTable(RowSortedTable<R, ? extends C, ? extends V> table) {
        return new UnmodifiableRowSortedMap(table);
    }

    static final class UnmodifiableRowSortedMap<R, C, V> extends UnmodifiableTable<R, C, V> implements RowSortedTable<R, C, V> {
        private static final long serialVersionUID = 0;

        public UnmodifiableRowSortedMap(RowSortedTable<R, ? extends C, ? extends V> delegate) {
            super(delegate);
        }

        /* access modifiers changed from: protected */
        public RowSortedTable<R, C, V> delegate() {
            return (RowSortedTable) super.delegate();
        }

        public SortedMap<R, Map<C, V>> rowMap() {
            return Collections.unmodifiableSortedMap(Maps.transformValues(delegate().rowMap(), Tables.unmodifiableWrapper()));
        }

        public SortedSet<R> rowKeySet() {
            return Collections.unmodifiableSortedSet(delegate().rowKeySet());
        }
    }

    /* access modifiers changed from: private */
    public static <K, V> Function<Map<K, V>, Map<K, V>> unmodifiableWrapper() {
        return UNMODIFIABLE_WRAPPER;
    }
}
