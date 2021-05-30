package com.google.common.collect;

import com.google.common.annotations.GwtCompatible;
import com.google.common.base.Preconditions;
import com.google.common.base.Supplier;
import java.util.Comparator;
import java.util.Map;
import java.util.SortedMap;
import java.util.SortedSet;

@GwtCompatible
class StandardRowSortedTable<R, C, V> extends StandardTable<R, C, V> implements RowSortedTable<R, C, V> {
    private static final long serialVersionUID = 0;
    private transient SortedSet<R> rowKeySet;
    private transient StandardRowSortedTable<R, C, V>.RowSortedMap rowMap;

    StandardRowSortedTable(SortedMap<R, Map<C, V>> backingMap, Supplier<? extends Map<C, V>> factory) {
        super(backingMap, factory);
    }

    /* access modifiers changed from: private */
    public SortedMap<R, Map<C, V>> sortedBackingMap() {
        return (SortedMap) this.backingMap;
    }

    public SortedSet<R> rowKeySet() {
        SortedSet<R> result = this.rowKeySet;
        if (result != null) {
            return result;
        }
        RowKeySortedSet rowKeySortedSet = new RowKeySortedSet();
        this.rowKeySet = rowKeySortedSet;
        return rowKeySortedSet;
    }

    private class RowKeySortedSet extends StandardTable<R, C, V>.RowKeySet implements SortedSet<R> {
        private RowKeySortedSet() {
            super();
        }

        public Comparator<? super R> comparator() {
            return StandardRowSortedTable.this.sortedBackingMap().comparator();
        }

        public R first() {
            return StandardRowSortedTable.this.sortedBackingMap().firstKey();
        }

        public R last() {
            return StandardRowSortedTable.this.sortedBackingMap().lastKey();
        }

        public SortedSet<R> headSet(R toElement) {
            Preconditions.checkNotNull(toElement);
            return new StandardRowSortedTable(StandardRowSortedTable.this.sortedBackingMap().headMap(toElement), StandardRowSortedTable.this.factory).rowKeySet();
        }

        public SortedSet<R> subSet(R fromElement, R toElement) {
            Preconditions.checkNotNull(fromElement);
            Preconditions.checkNotNull(toElement);
            return new StandardRowSortedTable(StandardRowSortedTable.this.sortedBackingMap().subMap(fromElement, toElement), StandardRowSortedTable.this.factory).rowKeySet();
        }

        public SortedSet<R> tailSet(R fromElement) {
            Preconditions.checkNotNull(fromElement);
            return new StandardRowSortedTable(StandardRowSortedTable.this.sortedBackingMap().tailMap(fromElement), StandardRowSortedTable.this.factory).rowKeySet();
        }
    }

    public SortedMap<R, Map<C, V>> rowMap() {
        StandardRowSortedTable<R, C, V>.RowSortedMap result = this.rowMap;
        if (result != null) {
            return result;
        }
        StandardRowSortedTable<R, C, V>.RowSortedMap rowSortedMap = new RowSortedMap();
        this.rowMap = rowSortedMap;
        return rowSortedMap;
    }

    private class RowSortedMap extends StandardTable<R, C, V>.RowMap implements SortedMap<R, Map<C, V>> {
        private RowSortedMap() {
            super();
        }

        public Comparator<? super R> comparator() {
            return StandardRowSortedTable.this.sortedBackingMap().comparator();
        }

        public R firstKey() {
            return StandardRowSortedTable.this.sortedBackingMap().firstKey();
        }

        public R lastKey() {
            return StandardRowSortedTable.this.sortedBackingMap().lastKey();
        }

        public SortedMap<R, Map<C, V>> headMap(R toKey) {
            Preconditions.checkNotNull(toKey);
            return new StandardRowSortedTable(StandardRowSortedTable.this.sortedBackingMap().headMap(toKey), StandardRowSortedTable.this.factory).rowMap();
        }

        public SortedMap<R, Map<C, V>> subMap(R fromKey, R toKey) {
            Preconditions.checkNotNull(fromKey);
            Preconditions.checkNotNull(toKey);
            return new StandardRowSortedTable(StandardRowSortedTable.this.sortedBackingMap().subMap(fromKey, toKey), StandardRowSortedTable.this.factory).rowMap();
        }

        public SortedMap<R, Map<C, V>> tailMap(R fromKey) {
            Preconditions.checkNotNull(fromKey);
            return new StandardRowSortedTable(StandardRowSortedTable.this.sortedBackingMap().tailMap(fromKey), StandardRowSortedTable.this.factory).rowMap();
        }
    }
}
