package com.google.common.collect;

import com.google.common.annotations.GwtCompatible;
import com.google.common.annotations.VisibleForTesting;
import com.google.common.base.Function;
import com.google.common.base.Objects;
import com.google.common.base.Preconditions;
import com.google.common.collect.ImmutableBiMap;
import com.google.common.collect.ImmutableMap;
import com.google.common.collect.ImmutableSet;
import com.google.common.collect.Table;
import java.lang.reflect.Array;
import java.util.Collection;
import java.util.Collections;
import java.util.Comparator;
import java.util.Iterator;
import java.util.List;
import java.util.Map;
import java.util.Set;
import javax.annotation.Nullable;
import javax.annotation.concurrent.Immutable;

@GwtCompatible
abstract class RegularImmutableTable<R, C, V> extends ImmutableTable<R, C, V> {
    private static final Function<Table.Cell<Object, Object, Object>, Object> GET_VALUE_FUNCTION = new Function<Table.Cell<Object, Object, Object>, Object>() {
        public Object apply(Table.Cell<Object, Object, Object> from) {
            return from.getValue();
        }
    };
    private final ImmutableSet<Table.Cell<R, C, V>> cellSet;
    @Nullable
    private volatile transient ImmutableList<V> valueList;

    private RegularImmutableTable(ImmutableSet<Table.Cell<R, C, V>> cellSet2) {
        this.cellSet = cellSet2;
    }

    private Function<Table.Cell<R, C, V>, V> getValueFunction() {
        return GET_VALUE_FUNCTION;
    }

    public final ImmutableCollection<V> values() {
        ImmutableList<V> result = this.valueList;
        if (result != null) {
            return result;
        }
        ImmutableList<V> copyOf = ImmutableList.copyOf(Iterables.transform(cellSet(), getValueFunction()));
        ImmutableList<V> result2 = copyOf;
        this.valueList = copyOf;
        return result2;
    }

    public final int size() {
        return cellSet().size();
    }

    public final boolean containsValue(@Nullable Object value) {
        return values().contains(value);
    }

    public final boolean isEmpty() {
        return false;
    }

    public final ImmutableSet<Table.Cell<R, C, V>> cellSet() {
        return this.cellSet;
    }

    static final <R, C, V> RegularImmutableTable<R, C, V> forCells(List<Table.Cell<R, C, V>> cells, @Nullable final Comparator<? super R> rowComparator, @Nullable final Comparator<? super C> columnComparator) {
        Preconditions.checkNotNull(cells);
        if (!(rowComparator == null && columnComparator == null)) {
            Collections.sort(cells, new Comparator<Table.Cell<R, C, V>>() {
                public int compare(Table.Cell<R, C, V> cell1, Table.Cell<R, C, V> cell2) {
                    int rowCompare = rowComparator == null ? 0 : rowComparator.compare(cell1.getRowKey(), cell2.getRowKey());
                    if (rowCompare != 0) {
                        return rowCompare;
                    }
                    if (columnComparator == null) {
                        return 0;
                    }
                    return columnComparator.compare(cell1.getColumnKey(), cell2.getColumnKey());
                }
            });
        }
        return forCellsInternal(cells, rowComparator, columnComparator);
    }

    static final <R, C, V> RegularImmutableTable<R, C, V> forCells(Iterable<Table.Cell<R, C, V>> cells) {
        return forCellsInternal(cells, (Comparator) null, (Comparator) null);
    }

    private static final <R, C, V> RegularImmutableTable<R, C, V> forCellsInternal(Iterable<Table.Cell<R, C, V>> cells, @Nullable Comparator<? super R> rowComparator, @Nullable Comparator<? super C> columnComparator) {
        ImmutableSet.Builder<Table.Cell<R, C, V>> cellSetBuilder = ImmutableSet.builder();
        ImmutableSet.Builder<R> rowSpaceBuilder = ImmutableSet.builder();
        ImmutableSet.Builder<C> columnSpaceBuilder = ImmutableSet.builder();
        for (Table.Cell<R, C, V> cell : cells) {
            cellSetBuilder.add((Object) cell);
            rowSpaceBuilder.add((Object) cell.getRowKey());
            columnSpaceBuilder.add((Object) cell.getColumnKey());
        }
        ImmutableSet<Table.Cell<R, C, V>> cellSet2 = cellSetBuilder.build();
        ImmutableSet<R> rowSpace = rowSpaceBuilder.build();
        if (rowComparator != null) {
            List<R> rowList = Lists.newArrayList(rowSpace);
            Collections.sort(rowList, rowComparator);
            rowSpace = ImmutableSet.copyOf(rowList);
        }
        ImmutableSet<C> columnSpace = columnSpaceBuilder.build();
        if (columnComparator != null) {
            List<C> columnList = Lists.newArrayList(columnSpace);
            Collections.sort(columnList, columnComparator);
            columnSpace = ImmutableSet.copyOf(columnList);
        }
        return cellSet2.size() > (rowSpace.size() * columnSpace.size()) / 2 ? new DenseImmutableTable(cellSet2, rowSpace, columnSpace) : new SparseImmutableTable(cellSet2, rowSpace, columnSpace);
    }

    @VisibleForTesting
    @Immutable
    static final class SparseImmutableTable<R, C, V> extends RegularImmutableTable<R, C, V> {
        private final ImmutableMap<C, Map<R, V>> columnMap;
        private final ImmutableMap<R, Map<C, V>> rowMap;

        public /* bridge */ /* synthetic */ Set cellSet() {
            return RegularImmutableTable.super.cellSet();
        }

        public /* bridge */ /* synthetic */ Collection values() {
            return RegularImmutableTable.super.values();
        }

        private static final <A, B, V> Map<A, ImmutableMap.Builder<B, V>> makeIndexBuilder(ImmutableSet<A> keySpace) {
            Map<A, ImmutableMap.Builder<B, V>> indexBuilder = Maps.newLinkedHashMap();
            Iterator i$ = keySpace.iterator();
            while (i$.hasNext()) {
                indexBuilder.put(i$.next(), ImmutableMap.builder());
            }
            return indexBuilder;
        }

        private static final <A, B, V> ImmutableMap<A, Map<B, V>> buildIndex(Map<A, ImmutableMap.Builder<B, V>> indexBuilder) {
            return ImmutableMap.copyOf(Maps.transformValues(indexBuilder, new Function<ImmutableMap.Builder<B, V>, Map<B, V>>() {
                public Map<B, V> apply(ImmutableMap.Builder<B, V> from) {
                    return from.build();
                }
            }));
        }

        SparseImmutableTable(ImmutableSet<Table.Cell<R, C, V>> cellSet, ImmutableSet<R> rowSpace, ImmutableSet<C> columnSpace) {
            super(cellSet);
            Map<R, ImmutableMap.Builder<C, V>> rowIndexBuilder = makeIndexBuilder(rowSpace);
            Map<C, ImmutableMap.Builder<R, V>> columnIndexBuilder = makeIndexBuilder(columnSpace);
            Iterator i$ = cellSet.iterator();
            while (i$.hasNext()) {
                Table.Cell<R, C, V> cell = i$.next();
                R rowKey = cell.getRowKey();
                C columnKey = cell.getColumnKey();
                V value = cell.getValue();
                rowIndexBuilder.get(rowKey).put(columnKey, value);
                columnIndexBuilder.get(columnKey).put(rowKey, value);
            }
            this.rowMap = buildIndex(rowIndexBuilder);
            this.columnMap = buildIndex(columnIndexBuilder);
        }

        public ImmutableMap<R, V> column(C columnKey) {
            Preconditions.checkNotNull(columnKey);
            return (ImmutableMap) Objects.firstNonNull((ImmutableMap) this.columnMap.get(columnKey), ImmutableMap.of());
        }

        public ImmutableSet<C> columnKeySet() {
            return this.columnMap.keySet();
        }

        public ImmutableMap<C, Map<R, V>> columnMap() {
            return this.columnMap;
        }

        public ImmutableMap<C, V> row(R rowKey) {
            Preconditions.checkNotNull(rowKey);
            return (ImmutableMap) Objects.firstNonNull((ImmutableMap) this.rowMap.get(rowKey), ImmutableMap.of());
        }

        public ImmutableSet<R> rowKeySet() {
            return this.rowMap.keySet();
        }

        public ImmutableMap<R, Map<C, V>> rowMap() {
            return this.rowMap;
        }

        public boolean contains(@Nullable Object rowKey, @Nullable Object columnKey) {
            Map<C, V> row = this.rowMap.get(rowKey);
            return row != null && row.containsKey(columnKey);
        }

        public boolean containsColumn(@Nullable Object columnKey) {
            return this.columnMap.containsKey(columnKey);
        }

        public boolean containsRow(@Nullable Object rowKey) {
            return this.rowMap.containsKey(rowKey);
        }

        public V get(@Nullable Object rowKey, @Nullable Object columnKey) {
            Map<C, V> row = this.rowMap.get(rowKey);
            if (row == null) {
                return null;
            }
            return row.get(columnKey);
        }
    }

    @VisibleForTesting
    @Immutable
    static final class DenseImmutableTable<R, C, V> extends RegularImmutableTable<R, C, V> {
        private final ImmutableBiMap<C, Integer> columnKeyToIndex;
        private volatile transient ImmutableMap<C, Map<R, V>> columnMap;
        private final ImmutableBiMap<R, Integer> rowKeyToIndex;
        private volatile transient ImmutableMap<R, Map<C, V>> rowMap;
        private final V[][] values;

        public /* bridge */ /* synthetic */ Set cellSet() {
            return RegularImmutableTable.super.cellSet();
        }

        public /* bridge */ /* synthetic */ Collection values() {
            return RegularImmutableTable.super.values();
        }

        private static <E> ImmutableBiMap<E, Integer> makeIndex(ImmutableSet<E> set) {
            ImmutableBiMap.Builder<E, Integer> indexBuilder = ImmutableBiMap.builder();
            int i = 0;
            Iterator i$ = set.iterator();
            while (i$.hasNext()) {
                indexBuilder.put(i$.next(), Integer.valueOf(i));
                i++;
            }
            return indexBuilder.build();
        }

        /* JADX INFO: super call moved to the top of the method (can break code semantics) */
        DenseImmutableTable(ImmutableSet<Table.Cell<R, C, V>> cellSet, ImmutableSet<R> rowSpace, ImmutableSet<C> columnSpace) {
            super(cellSet);
            this.values = (Object[][]) Array.newInstance(Object.class, new int[]{rowSpace.size(), columnSpace.size()});
            this.rowKeyToIndex = makeIndex(rowSpace);
            this.columnKeyToIndex = makeIndex(columnSpace);
            Iterator i$ = cellSet.iterator();
            while (i$.hasNext()) {
                Table.Cell<R, C, V> cell = i$.next();
                R rowKey = cell.getRowKey();
                C columnKey = cell.getColumnKey();
                int rowIndex = this.rowKeyToIndex.get(rowKey).intValue();
                int columnIndex = this.columnKeyToIndex.get(columnKey).intValue();
                Preconditions.checkArgument(this.values[rowIndex][columnIndex] == null, "duplicate key: (%s, %s)", rowKey, columnKey);
                this.values[rowIndex][columnIndex] = cell.getValue();
            }
        }

        public ImmutableMap<R, V> column(C columnKey) {
            Preconditions.checkNotNull(columnKey);
            Integer columnIndexInteger = this.columnKeyToIndex.get(columnKey);
            if (columnIndexInteger == null) {
                return ImmutableMap.of();
            }
            int columnIndex = columnIndexInteger.intValue();
            ImmutableMap.Builder<R, V> columnBuilder = ImmutableMap.builder();
            for (int i = 0; i < this.values.length; i++) {
                V value = this.values[i][columnIndex];
                if (value != null) {
                    columnBuilder.put(this.rowKeyToIndex.inverse().get(Integer.valueOf(i)), value);
                }
            }
            return columnBuilder.build();
        }

        public ImmutableSet<C> columnKeySet() {
            return this.columnKeyToIndex.keySet();
        }

        private ImmutableMap<C, Map<R, V>> makeColumnMap() {
            ImmutableMap.Builder<C, Map<R, V>> columnMapBuilder = ImmutableMap.builder();
            for (int c = 0; c < this.columnKeyToIndex.size(); c++) {
                ImmutableMap.Builder<R, V> rowMapBuilder = ImmutableMap.builder();
                for (int r = 0; r < this.rowKeyToIndex.size(); r++) {
                    V value = this.values[r][c];
                    if (value != null) {
                        rowMapBuilder.put(this.rowKeyToIndex.inverse().get(Integer.valueOf(r)), value);
                    }
                }
                columnMapBuilder.put(this.columnKeyToIndex.inverse().get(Integer.valueOf(c)), rowMapBuilder.build());
            }
            return columnMapBuilder.build();
        }

        public ImmutableMap<C, Map<R, V>> columnMap() {
            ImmutableMap<C, Map<R, V>> result = this.columnMap;
            if (result != null) {
                return result;
            }
            ImmutableMap<C, Map<R, V>> makeColumnMap = makeColumnMap();
            ImmutableMap<C, Map<R, V>> result2 = makeColumnMap;
            this.columnMap = makeColumnMap;
            return result2;
        }

        public boolean contains(@Nullable Object rowKey, @Nullable Object columnKey) {
            return get(rowKey, columnKey) != null;
        }

        public boolean containsColumn(@Nullable Object columnKey) {
            return this.columnKeyToIndex.containsKey(columnKey);
        }

        public boolean containsRow(@Nullable Object rowKey) {
            return this.rowKeyToIndex.containsKey(rowKey);
        }

        public V get(@Nullable Object rowKey, @Nullable Object columnKey) {
            Integer rowIndex = this.rowKeyToIndex.get(rowKey);
            Integer columnIndex = this.columnKeyToIndex.get(columnKey);
            if (rowIndex == null || columnIndex == null) {
                return null;
            }
            return this.values[rowIndex.intValue()][columnIndex.intValue()];
        }

        public ImmutableMap<C, V> row(R rowKey) {
            Preconditions.checkNotNull(rowKey);
            Integer rowIndex = this.rowKeyToIndex.get(rowKey);
            if (rowIndex == null) {
                return ImmutableMap.of();
            }
            ImmutableMap.Builder<C, V> rowBuilder = ImmutableMap.builder();
            V[] row = this.values[rowIndex.intValue()];
            for (int r = 0; r < row.length; r++) {
                V value = row[r];
                if (value != null) {
                    rowBuilder.put(this.columnKeyToIndex.inverse().get(Integer.valueOf(r)), value);
                }
            }
            return rowBuilder.build();
        }

        public ImmutableSet<R> rowKeySet() {
            return this.rowKeyToIndex.keySet();
        }

        private ImmutableMap<R, Map<C, V>> makeRowMap() {
            ImmutableMap.Builder<R, Map<C, V>> rowMapBuilder = ImmutableMap.builder();
            for (int r = 0; r < this.values.length; r++) {
                V[] row = this.values[r];
                ImmutableMap.Builder<C, V> columnMapBuilder = ImmutableMap.builder();
                for (int c = 0; c < row.length; c++) {
                    V value = row[c];
                    if (value != null) {
                        columnMapBuilder.put(this.columnKeyToIndex.inverse().get(Integer.valueOf(c)), value);
                    }
                }
                rowMapBuilder.put(this.rowKeyToIndex.inverse().get(Integer.valueOf(r)), columnMapBuilder.build());
            }
            return rowMapBuilder.build();
        }

        public ImmutableMap<R, Map<C, V>> rowMap() {
            ImmutableMap<R, Map<C, V>> result = this.rowMap;
            if (result != null) {
                return result;
            }
            ImmutableMap<R, Map<C, V>> makeRowMap = makeRowMap();
            ImmutableMap<R, Map<C, V>> result2 = makeRowMap;
            this.rowMap = makeRowMap;
            return result2;
        }
    }
}
