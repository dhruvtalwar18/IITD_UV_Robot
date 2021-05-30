package com.google.common.collect;

import com.google.common.annotations.GwtCompatible;
import com.google.common.base.Preconditions;
import com.google.common.base.Predicate;
import com.google.common.base.Predicates;
import com.google.common.base.Supplier;
import com.google.common.collect.Maps;
import com.google.common.collect.Table;
import java.io.Serializable;
import java.util.AbstractCollection;
import java.util.AbstractMap;
import java.util.AbstractSet;
import java.util.Collection;
import java.util.Iterator;
import java.util.LinkedHashMap;
import java.util.Map;
import java.util.Set;
import javax.annotation.Nullable;

@GwtCompatible
class StandardTable<R, C, V> implements Table<R, C, V>, Serializable {
    private static final long serialVersionUID = 0;
    @GwtTransient
    final Map<R, Map<C, V>> backingMap;
    private transient StandardTable<R, C, V>.CellSet cellSet;
    private transient Set<C> columnKeySet;
    private transient StandardTable<R, C, V>.ColumnMap columnMap;
    @GwtTransient
    final Supplier<? extends Map<C, V>> factory;
    private transient StandardTable<R, C, V>.RowKeySet rowKeySet;
    private transient StandardTable<R, C, V>.RowMap rowMap;
    private transient StandardTable<R, C, V>.Values values;

    StandardTable(Map<R, Map<C, V>> backingMap2, Supplier<? extends Map<C, V>> factory2) {
        this.backingMap = backingMap2;
        this.factory = factory2;
    }

    public boolean contains(@Nullable Object rowKey, @Nullable Object columnKey) {
        Map<C, V> map;
        if (rowKey == null || columnKey == null || (map = (Map) Maps.safeGet(this.backingMap, rowKey)) == null || !Maps.safeContainsKey(map, columnKey)) {
            return false;
        }
        return true;
    }

    public boolean containsColumn(@Nullable Object columnKey) {
        if (columnKey == null) {
            return false;
        }
        for (Map<C, V> map : this.backingMap.values()) {
            if (Maps.safeContainsKey(map, columnKey)) {
                return true;
            }
        }
        return false;
    }

    public boolean containsRow(@Nullable Object rowKey) {
        return rowKey != null && Maps.safeContainsKey(this.backingMap, rowKey);
    }

    public boolean containsValue(@Nullable Object value) {
        if (value == null) {
            return false;
        }
        for (Map<C, V> map : this.backingMap.values()) {
            if (map.containsValue(value)) {
                return true;
            }
        }
        return false;
    }

    public V get(@Nullable Object rowKey, @Nullable Object columnKey) {
        Map<C, V> map;
        if (rowKey == null || columnKey == null || (map = (Map) Maps.safeGet(this.backingMap, rowKey)) == null) {
            return null;
        }
        return Maps.safeGet(map, columnKey);
    }

    public boolean isEmpty() {
        return this.backingMap.isEmpty();
    }

    public int size() {
        int size = 0;
        for (Map<C, V> map : this.backingMap.values()) {
            size += map.size();
        }
        return size;
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

    public void clear() {
        this.backingMap.clear();
    }

    private Map<C, V> getOrCreate(R rowKey) {
        Map<C, V> map = this.backingMap.get(rowKey);
        if (map != null) {
            return map;
        }
        Map<C, V> map2 = (Map) this.factory.get();
        this.backingMap.put(rowKey, map2);
        return map2;
    }

    public V put(R rowKey, C columnKey, V value) {
        Preconditions.checkNotNull(rowKey);
        Preconditions.checkNotNull(columnKey);
        Preconditions.checkNotNull(value);
        return getOrCreate(rowKey).put(columnKey, value);
    }

    public void putAll(Table<? extends R, ? extends C, ? extends V> table) {
        for (Table.Cell<? extends R, ? extends C, ? extends V> cell : table.cellSet()) {
            put(cell.getRowKey(), cell.getColumnKey(), cell.getValue());
        }
    }

    public V remove(@Nullable Object rowKey, @Nullable Object columnKey) {
        Map<C, V> map;
        if (rowKey == null || columnKey == null || (map = (Map) Maps.safeGet(this.backingMap, rowKey)) == null) {
            return null;
        }
        V value = map.remove(columnKey);
        if (map.isEmpty()) {
            this.backingMap.remove(rowKey);
        }
        return value;
    }

    /* access modifiers changed from: private */
    public Map<R, V> removeColumn(Object column) {
        Map<R, V> output = new LinkedHashMap<>();
        Iterator<Map.Entry<R, Map<C, V>>> iterator = this.backingMap.entrySet().iterator();
        while (iterator.hasNext()) {
            Map.Entry<R, Map<C, V>> entry = iterator.next();
            V value = entry.getValue().remove(column);
            if (value != null) {
                output.put(entry.getKey(), value);
                if (entry.getValue().isEmpty()) {
                    iterator.remove();
                }
            }
        }
        return output;
    }

    /* access modifiers changed from: private */
    public boolean containsMapping(Object rowKey, Object columnKey, Object value) {
        return value != null && value.equals(get(rowKey, columnKey));
    }

    /* access modifiers changed from: private */
    public boolean removeMapping(Object rowKey, Object columnKey, Object value) {
        if (!containsMapping(rowKey, columnKey, value)) {
            return false;
        }
        remove(rowKey, columnKey);
        return true;
    }

    private abstract class TableCollection<T> extends AbstractCollection<T> {
        private TableCollection() {
        }

        public boolean isEmpty() {
            return StandardTable.this.backingMap.isEmpty();
        }

        public void clear() {
            StandardTable.this.backingMap.clear();
        }
    }

    private abstract class TableSet<T> extends AbstractSet<T> {
        private TableSet() {
        }

        public boolean isEmpty() {
            return StandardTable.this.backingMap.isEmpty();
        }

        public void clear() {
            StandardTable.this.backingMap.clear();
        }
    }

    public Set<Table.Cell<R, C, V>> cellSet() {
        StandardTable<R, C, V>.CellSet result = this.cellSet;
        if (result != null) {
            return result;
        }
        StandardTable<R, C, V>.CellSet cellSet2 = new CellSet();
        this.cellSet = cellSet2;
        return cellSet2;
    }

    private class CellSet extends StandardTable<R, C, V>.TableSet<Table.Cell<R, C, V>> {
        private CellSet() {
            super();
        }

        public Iterator<Table.Cell<R, C, V>> iterator() {
            return new CellIterator();
        }

        public int size() {
            return StandardTable.this.size();
        }

        public boolean contains(Object obj) {
            if (!(obj instanceof Table.Cell)) {
                return false;
            }
            Table.Cell<?, ?, ?> cell = (Table.Cell) obj;
            return StandardTable.this.containsMapping(cell.getRowKey(), cell.getColumnKey(), cell.getValue());
        }

        public boolean remove(Object obj) {
            if (!(obj instanceof Table.Cell)) {
                return false;
            }
            Table.Cell<?, ?, ?> cell = (Table.Cell) obj;
            return StandardTable.this.removeMapping(cell.getRowKey(), cell.getColumnKey(), cell.getValue());
        }
    }

    private class CellIterator implements Iterator<Table.Cell<R, C, V>> {
        Iterator<Map.Entry<C, V>> columnIterator;
        Map.Entry<R, Map<C, V>> rowEntry;
        final Iterator<Map.Entry<R, Map<C, V>>> rowIterator;

        private CellIterator() {
            this.rowIterator = StandardTable.this.backingMap.entrySet().iterator();
            this.columnIterator = Iterators.emptyModifiableIterator();
        }

        public boolean hasNext() {
            return this.rowIterator.hasNext() || this.columnIterator.hasNext();
        }

        public Table.Cell<R, C, V> next() {
            if (!this.columnIterator.hasNext()) {
                this.rowEntry = this.rowIterator.next();
                this.columnIterator = this.rowEntry.getValue().entrySet().iterator();
            }
            Map.Entry<C, V> columnEntry = this.columnIterator.next();
            return Tables.immutableCell(this.rowEntry.getKey(), columnEntry.getKey(), columnEntry.getValue());
        }

        public void remove() {
            this.columnIterator.remove();
            if (this.rowEntry.getValue().isEmpty()) {
                this.rowIterator.remove();
            }
        }
    }

    public Map<C, V> row(R rowKey) {
        return new Row(rowKey);
    }

    class Row extends AbstractMap<C, V> {
        Map<C, V> backingRowMap;
        Set<Map.Entry<C, V>> entrySet;
        Set<C> keySet;
        final R rowKey;

        Row(R rowKey2) {
            this.rowKey = Preconditions.checkNotNull(rowKey2);
        }

        /* access modifiers changed from: package-private */
        public Map<C, V> backingRowMap() {
            if (this.backingRowMap != null && (!this.backingRowMap.isEmpty() || !StandardTable.this.backingMap.containsKey(this.rowKey))) {
                return this.backingRowMap;
            }
            Map<C, V> computeBackingRowMap = computeBackingRowMap();
            this.backingRowMap = computeBackingRowMap;
            return computeBackingRowMap;
        }

        /* access modifiers changed from: package-private */
        public Map<C, V> computeBackingRowMap() {
            return StandardTable.this.backingMap.get(this.rowKey);
        }

        /* access modifiers changed from: package-private */
        public void maintainEmptyInvariant() {
            if (backingRowMap() != null && this.backingRowMap.isEmpty()) {
                StandardTable.this.backingMap.remove(this.rowKey);
                this.backingRowMap = null;
            }
        }

        public boolean containsKey(Object key) {
            Map<C, V> backingRowMap2 = backingRowMap();
            return (key == null || backingRowMap2 == null || !Maps.safeContainsKey(backingRowMap2, key)) ? false : true;
        }

        public V get(Object key) {
            Map<C, V> backingRowMap2 = backingRowMap();
            if (key == null || backingRowMap2 == null) {
                return null;
            }
            return Maps.safeGet(backingRowMap2, key);
        }

        public V put(C key, V value) {
            Preconditions.checkNotNull(key);
            Preconditions.checkNotNull(value);
            if (this.backingRowMap == null || this.backingRowMap.isEmpty()) {
                return StandardTable.this.put(this.rowKey, key, value);
            }
            return this.backingRowMap.put(key, value);
        }

        public V remove(Object key) {
            try {
                Map<C, V> backingRowMap2 = backingRowMap();
                if (backingRowMap2 == null) {
                    return null;
                }
                V result = backingRowMap2.remove(key);
                maintainEmptyInvariant();
                return result;
            } catch (ClassCastException e) {
                return null;
            }
        }

        public void clear() {
            Map<C, V> backingRowMap2 = backingRowMap();
            if (backingRowMap2 != null) {
                backingRowMap2.clear();
            }
            maintainEmptyInvariant();
        }

        public Set<C> keySet() {
            Set<C> result = this.keySet;
            if (result != null) {
                return result;
            }
            AnonymousClass1 r1 = new Maps.KeySet<C, V>() {
                /* access modifiers changed from: package-private */
                public Map<C, V> map() {
                    return Row.this;
                }
            };
            this.keySet = r1;
            return r1;
        }

        public Set<Map.Entry<C, V>> entrySet() {
            Set<Map.Entry<C, V>> result = this.entrySet;
            if (result != null) {
                return result;
            }
            RowEntrySet rowEntrySet = new RowEntrySet();
            this.entrySet = rowEntrySet;
            return rowEntrySet;
        }

        private class RowEntrySet extends Maps.EntrySet<C, V> {
            private RowEntrySet() {
            }

            /* access modifiers changed from: package-private */
            public Map<C, V> map() {
                return Row.this;
            }

            public int size() {
                Map<C, V> map = Row.this.backingRowMap();
                if (map == null) {
                    return 0;
                }
                return map.size();
            }

            public Iterator<Map.Entry<C, V>> iterator() {
                Map<C, V> map = Row.this.backingRowMap();
                if (map == null) {
                    return Iterators.emptyModifiableIterator();
                }
                final Iterator<Map.Entry<C, V>> iterator = map.entrySet().iterator();
                return new Iterator<Map.Entry<C, V>>() {
                    public boolean hasNext() {
                        return iterator.hasNext();
                    }

                    public Map.Entry<C, V> next() {
                        final Map.Entry<C, V> entry = (Map.Entry) iterator.next();
                        return new ForwardingMapEntry<C, V>() {
                            /* access modifiers changed from: protected */
                            public Map.Entry<C, V> delegate() {
                                return entry;
                            }

                            public V setValue(V value) {
                                return super.setValue(Preconditions.checkNotNull(value));
                            }

                            public boolean equals(Object object) {
                                return standardEquals(object);
                            }
                        };
                    }

                    public void remove() {
                        iterator.remove();
                        Row.this.maintainEmptyInvariant();
                    }
                };
            }
        }
    }

    public Map<R, V> column(C columnKey) {
        return new Column(columnKey);
    }

    private class Column extends Maps.ImprovedAbstractMap<R, V> {
        final C columnKey;
        StandardTable<R, C, V>.Column.Values columnValues;
        StandardTable<R, C, V>.Column.KeySet keySet;

        Column(C columnKey2) {
            this.columnKey = Preconditions.checkNotNull(columnKey2);
        }

        public V put(R key, V value) {
            return StandardTable.this.put(key, this.columnKey, value);
        }

        public V get(Object key) {
            return StandardTable.this.get(key, this.columnKey);
        }

        public boolean containsKey(Object key) {
            return StandardTable.this.contains(key, this.columnKey);
        }

        public V remove(Object key) {
            return StandardTable.this.remove(key, this.columnKey);
        }

        public Set<Map.Entry<R, V>> createEntrySet() {
            return new EntrySet();
        }

        public Collection<V> values() {
            StandardTable<R, C, V>.Column.Values result = this.columnValues;
            if (result != null) {
                return result;
            }
            Values values = new Values();
            this.columnValues = values;
            return values;
        }

        /* access modifiers changed from: package-private */
        public boolean removePredicate(Predicate<? super Map.Entry<R, V>> predicate) {
            boolean changed = false;
            Iterator<Map.Entry<R, Map<C, V>>> iterator = StandardTable.this.backingMap.entrySet().iterator();
            while (iterator.hasNext()) {
                Map.Entry<R, Map<C, V>> entry = iterator.next();
                Map<C, V> map = entry.getValue();
                V value = map.get(this.columnKey);
                if (value != null && predicate.apply(new ImmutableEntry(entry.getKey(), value))) {
                    map.remove(this.columnKey);
                    changed = true;
                    if (map.isEmpty()) {
                        iterator.remove();
                    }
                }
            }
            return changed;
        }

        class EntrySet extends AbstractSet<Map.Entry<R, V>> {
            EntrySet() {
            }

            public Iterator<Map.Entry<R, V>> iterator() {
                return new EntrySetIterator();
            }

            public int size() {
                int size = 0;
                for (Map<C, V> map : StandardTable.this.backingMap.values()) {
                    if (map.containsKey(Column.this.columnKey)) {
                        size++;
                    }
                }
                return size;
            }

            public boolean isEmpty() {
                return !StandardTable.this.containsColumn(Column.this.columnKey);
            }

            public void clear() {
                Column.this.removePredicate(Predicates.alwaysTrue());
            }

            public boolean contains(Object o) {
                if (!(o instanceof Map.Entry)) {
                    return false;
                }
                Map.Entry<?, ?> entry = (Map.Entry) o;
                return StandardTable.this.containsMapping(entry.getKey(), Column.this.columnKey, entry.getValue());
            }

            public boolean remove(Object obj) {
                if (!(obj instanceof Map.Entry)) {
                    return false;
                }
                Map.Entry<?, ?> entry = (Map.Entry) obj;
                return StandardTable.this.removeMapping(entry.getKey(), Column.this.columnKey, entry.getValue());
            }

            public boolean removeAll(Collection<?> c) {
                boolean changed = false;
                for (Object obj : c) {
                    changed |= remove(obj);
                }
                return changed;
            }

            public boolean retainAll(Collection<?> c) {
                return Column.this.removePredicate(Predicates.not(Predicates.in(c)));
            }
        }

        class EntrySetIterator extends AbstractIterator<Map.Entry<R, V>> {
            final Iterator<Map.Entry<R, Map<C, V>>> iterator = StandardTable.this.backingMap.entrySet().iterator();

            EntrySetIterator() {
            }

            /* access modifiers changed from: protected */
            public Map.Entry<R, V> computeNext() {
                while (this.iterator.hasNext()) {
                    final Map.Entry<R, Map<C, V>> entry = this.iterator.next();
                    if (entry.getValue().containsKey(Column.this.columnKey)) {
                        return new AbstractMapEntry<R, V>() {
                            public R getKey() {
                                return entry.getKey();
                            }

                            public V getValue() {
                                return ((Map) entry.getValue()).get(Column.this.columnKey);
                            }

                            public V setValue(V value) {
                                return ((Map) entry.getValue()).put(Column.this.columnKey, Preconditions.checkNotNull(value));
                            }
                        };
                    }
                }
                return (Map.Entry) endOfData();
            }
        }

        public Set<R> keySet() {
            StandardTable<R, C, V>.Column.KeySet result = this.keySet;
            if (result != null) {
                return result;
            }
            KeySet keySet2 = new KeySet();
            this.keySet = keySet2;
            return keySet2;
        }

        class KeySet extends AbstractSet<R> {
            KeySet() {
            }

            public Iterator<R> iterator() {
                return Maps.keyIterator(Column.this.entrySet().iterator());
            }

            public int size() {
                return Column.this.entrySet().size();
            }

            public boolean isEmpty() {
                return !StandardTable.this.containsColumn(Column.this.columnKey);
            }

            public boolean contains(Object obj) {
                return StandardTable.this.contains(obj, Column.this.columnKey);
            }

            public boolean remove(Object obj) {
                return StandardTable.this.remove(obj, Column.this.columnKey) != null;
            }

            public void clear() {
                Column.this.entrySet().clear();
            }

            public boolean removeAll(Collection<?> c) {
                boolean changed = false;
                for (Object obj : c) {
                    changed |= remove(obj);
                }
                return changed;
            }

            public boolean retainAll(final Collection<?> c) {
                Preconditions.checkNotNull(c);
                return Column.this.removePredicate(new Predicate<Map.Entry<R, V>>() {
                    public boolean apply(Map.Entry<R, V> entry) {
                        return !c.contains(entry.getKey());
                    }
                });
            }
        }

        class Values extends AbstractCollection<V> {
            Values() {
            }

            public Iterator<V> iterator() {
                return Maps.valueIterator(Column.this.entrySet().iterator());
            }

            public int size() {
                return Column.this.entrySet().size();
            }

            public boolean isEmpty() {
                return !StandardTable.this.containsColumn(Column.this.columnKey);
            }

            public void clear() {
                Column.this.entrySet().clear();
            }

            public boolean remove(Object obj) {
                if (obj == null) {
                    return false;
                }
                Iterator<Map<C, V>> iterator = StandardTable.this.backingMap.values().iterator();
                while (iterator.hasNext()) {
                    Map<C, V> map = iterator.next();
                    if (map.entrySet().remove(new ImmutableEntry(Column.this.columnKey, obj))) {
                        if (!map.isEmpty()) {
                            return true;
                        }
                        iterator.remove();
                        return true;
                    }
                }
                return false;
            }

            public boolean removeAll(final Collection<?> c) {
                Preconditions.checkNotNull(c);
                return Column.this.removePredicate(new Predicate<Map.Entry<R, V>>() {
                    public boolean apply(Map.Entry<R, V> entry) {
                        return c.contains(entry.getValue());
                    }
                });
            }

            public boolean retainAll(final Collection<?> c) {
                Preconditions.checkNotNull(c);
                return Column.this.removePredicate(new Predicate<Map.Entry<R, V>>() {
                    public boolean apply(Map.Entry<R, V> entry) {
                        return !c.contains(entry.getValue());
                    }
                });
            }
        }
    }

    public Set<R> rowKeySet() {
        Set<R> result = this.rowKeySet;
        if (result != null) {
            return result;
        }
        StandardTable<R, C, V>.RowKeySet rowKeySet2 = new RowKeySet();
        this.rowKeySet = rowKeySet2;
        return rowKeySet2;
    }

    class RowKeySet extends StandardTable<R, C, V>.TableSet<R> {
        RowKeySet() {
            super();
        }

        public Iterator<R> iterator() {
            return Maps.keyIterator(StandardTable.this.rowMap().entrySet().iterator());
        }

        public int size() {
            return StandardTable.this.backingMap.size();
        }

        public boolean contains(Object obj) {
            return StandardTable.this.containsRow(obj);
        }

        public boolean remove(Object obj) {
            return (obj == null || StandardTable.this.backingMap.remove(obj) == null) ? false : true;
        }
    }

    public Set<C> columnKeySet() {
        Set<C> result = this.columnKeySet;
        if (result != null) {
            return result;
        }
        ColumnKeySet columnKeySet2 = new ColumnKeySet();
        this.columnKeySet = columnKeySet2;
        return columnKeySet2;
    }

    private class ColumnKeySet extends StandardTable<R, C, V>.TableSet<C> {
        private ColumnKeySet() {
            super();
        }

        public Iterator<C> iterator() {
            return StandardTable.this.createColumnKeyIterator();
        }

        public int size() {
            return Iterators.size(iterator());
        }

        public boolean remove(Object obj) {
            if (obj == null) {
                return false;
            }
            boolean changed = false;
            Iterator<Map<C, V>> iterator = StandardTable.this.backingMap.values().iterator();
            while (iterator.hasNext()) {
                Map<C, V> map = iterator.next();
                if (map.keySet().remove(obj)) {
                    changed = true;
                    if (map.isEmpty()) {
                        iterator.remove();
                    }
                }
            }
            return changed;
        }

        public boolean removeAll(Collection<?> c) {
            Preconditions.checkNotNull(c);
            boolean changed = false;
            Iterator<Map<C, V>> iterator = StandardTable.this.backingMap.values().iterator();
            while (iterator.hasNext()) {
                Map<C, V> map = iterator.next();
                if (Iterators.removeAll(map.keySet().iterator(), c)) {
                    changed = true;
                    if (map.isEmpty()) {
                        iterator.remove();
                    }
                }
            }
            return changed;
        }

        public boolean retainAll(Collection<?> c) {
            Preconditions.checkNotNull(c);
            boolean changed = false;
            Iterator<Map<C, V>> iterator = StandardTable.this.backingMap.values().iterator();
            while (iterator.hasNext()) {
                Map<C, V> map = iterator.next();
                if (map.keySet().retainAll(c)) {
                    changed = true;
                    if (map.isEmpty()) {
                        iterator.remove();
                    }
                }
            }
            return changed;
        }

        public boolean contains(Object obj) {
            if (obj == null) {
                return false;
            }
            for (Map<C, V> map : StandardTable.this.backingMap.values()) {
                if (map.containsKey(obj)) {
                    return true;
                }
            }
            return false;
        }
    }

    /* access modifiers changed from: package-private */
    public Iterator<C> createColumnKeyIterator() {
        return new ColumnKeyIterator();
    }

    private class ColumnKeyIterator extends AbstractIterator<C> {
        Iterator<Map.Entry<C, V>> entryIterator;
        final Iterator<Map<C, V>> mapIterator;
        final Map<C, V> seen;

        private ColumnKeyIterator() {
            this.seen = (Map) StandardTable.this.factory.get();
            this.mapIterator = StandardTable.this.backingMap.values().iterator();
            this.entryIterator = Iterators.emptyIterator();
        }

        /* access modifiers changed from: protected */
        public C computeNext() {
            while (true) {
                if (this.entryIterator.hasNext()) {
                    Map.Entry<C, V> entry = this.entryIterator.next();
                    if (!this.seen.containsKey(entry.getKey())) {
                        this.seen.put(entry.getKey(), entry.getValue());
                        return entry.getKey();
                    }
                } else if (!this.mapIterator.hasNext()) {
                    return endOfData();
                } else {
                    this.entryIterator = this.mapIterator.next().entrySet().iterator();
                }
            }
        }
    }

    public Collection<V> values() {
        StandardTable<R, C, V>.Values result = this.values;
        if (result != null) {
            return result;
        }
        StandardTable<R, C, V>.Values values2 = new Values();
        this.values = values2;
        return values2;
    }

    private class Values extends StandardTable<R, C, V>.TableCollection<V> {
        private Values() {
            super();
        }

        public Iterator<V> iterator() {
            return new TransformedIterator<Table.Cell<R, C, V>, V>(StandardTable.this.cellSet().iterator()) {
                /* access modifiers changed from: package-private */
                public V transform(Table.Cell<R, C, V> cell) {
                    return cell.getValue();
                }
            };
        }

        public int size() {
            return StandardTable.this.size();
        }
    }

    public Map<R, Map<C, V>> rowMap() {
        StandardTable<R, C, V>.RowMap result = this.rowMap;
        if (result != null) {
            return result;
        }
        StandardTable<R, C, V>.RowMap rowMap2 = new RowMap();
        this.rowMap = rowMap2;
        return rowMap2;
    }

    class RowMap extends Maps.ImprovedAbstractMap<R, Map<C, V>> {
        RowMap() {
        }

        public boolean containsKey(Object key) {
            return StandardTable.this.containsRow(key);
        }

        public Map<C, V> get(Object key) {
            if (StandardTable.this.containsRow(key)) {
                return StandardTable.this.row(key);
            }
            return null;
        }

        public Set<R> keySet() {
            return StandardTable.this.rowKeySet();
        }

        public Map<C, V> remove(Object key) {
            if (key == null) {
                return null;
            }
            return StandardTable.this.backingMap.remove(key);
        }

        /* access modifiers changed from: protected */
        public Set<Map.Entry<R, Map<C, V>>> createEntrySet() {
            return new EntrySet();
        }

        class EntrySet extends StandardTable<R, C, V>.TableSet<Map.Entry<R, Map<C, V>>> {
            EntrySet() {
                super();
            }

            public Iterator<Map.Entry<R, Map<C, V>>> iterator() {
                return new TransformedIterator<R, Map.Entry<R, Map<C, V>>>(StandardTable.this.backingMap.keySet().iterator()) {
                    /* access modifiers changed from: package-private */
                    public Map.Entry<R, Map<C, V>> transform(R rowKey) {
                        return new ImmutableEntry(rowKey, StandardTable.this.row(rowKey));
                    }
                };
            }

            public int size() {
                return StandardTable.this.backingMap.size();
            }

            public boolean contains(Object obj) {
                if (!(obj instanceof Map.Entry)) {
                    return false;
                }
                Map.Entry<?, ?> entry = (Map.Entry) obj;
                if (entry.getKey() == null || !(entry.getValue() instanceof Map) || !Collections2.safeContains(StandardTable.this.backingMap.entrySet(), entry)) {
                    return false;
                }
                return true;
            }

            public boolean remove(Object obj) {
                if (!(obj instanceof Map.Entry)) {
                    return false;
                }
                Map.Entry<?, ?> entry = (Map.Entry) obj;
                if (entry.getKey() == null || !(entry.getValue() instanceof Map) || !StandardTable.this.backingMap.entrySet().remove(entry)) {
                    return false;
                }
                return true;
            }
        }
    }

    public Map<C, Map<R, V>> columnMap() {
        StandardTable<R, C, V>.ColumnMap result = this.columnMap;
        if (result != null) {
            return result;
        }
        StandardTable<R, C, V>.ColumnMap columnMap2 = new ColumnMap();
        this.columnMap = columnMap2;
        return columnMap2;
    }

    private class ColumnMap extends Maps.ImprovedAbstractMap<C, Map<R, V>> {
        StandardTable<R, C, V>.ColumnMap.ColumnMapValues columnMapValues;

        private ColumnMap() {
        }

        public Map<R, V> get(Object key) {
            if (StandardTable.this.containsColumn(key)) {
                return StandardTable.this.column(key);
            }
            return null;
        }

        public boolean containsKey(Object key) {
            return StandardTable.this.containsColumn(key);
        }

        public Map<R, V> remove(Object key) {
            if (StandardTable.this.containsColumn(key)) {
                return StandardTable.this.removeColumn(key);
            }
            return null;
        }

        public Set<Map.Entry<C, Map<R, V>>> createEntrySet() {
            return new ColumnMapEntrySet();
        }

        public Set<C> keySet() {
            return StandardTable.this.columnKeySet();
        }

        public Collection<Map<R, V>> values() {
            StandardTable<R, C, V>.ColumnMap.ColumnMapValues result = this.columnMapValues;
            if (result != null) {
                return result;
            }
            ColumnMapValues columnMapValues2 = new ColumnMapValues();
            this.columnMapValues = columnMapValues2;
            return columnMapValues2;
        }

        class ColumnMapEntrySet extends StandardTable<R, C, V>.TableSet<Map.Entry<C, Map<R, V>>> {
            ColumnMapEntrySet() {
                super();
            }

            public Iterator<Map.Entry<C, Map<R, V>>> iterator() {
                return new TransformedIterator<C, Map.Entry<C, Map<R, V>>>(StandardTable.this.columnKeySet().iterator()) {
                    /* access modifiers changed from: package-private */
                    public Map.Entry<C, Map<R, V>> transform(C columnKey) {
                        return new ImmutableEntry(columnKey, StandardTable.this.column(columnKey));
                    }
                };
            }

            public int size() {
                return StandardTable.this.columnKeySet().size();
            }

            public boolean contains(Object obj) {
                if (!(obj instanceof Map.Entry)) {
                    return false;
                }
                Map.Entry<?, ?> entry = (Map.Entry) obj;
                if (!StandardTable.this.containsColumn(entry.getKey())) {
                    return false;
                }
                return ColumnMap.this.get((Object) entry.getKey()).equals(entry.getValue());
            }

            public boolean remove(Object obj) {
                if (!contains(obj)) {
                    return false;
                }
                Map unused = StandardTable.this.removeColumn(((Map.Entry) obj).getKey());
                return true;
            }

            public boolean removeAll(Collection<?> c) {
                boolean changed = false;
                for (Object obj : c) {
                    changed |= remove(obj);
                }
                return changed;
            }

            public boolean retainAll(Collection<?> c) {
                boolean changed = false;
                Iterator i$ = Lists.newArrayList(StandardTable.this.columnKeySet().iterator()).iterator();
                while (i$.hasNext()) {
                    C columnKey = i$.next();
                    if (!c.contains(new ImmutableEntry(columnKey, StandardTable.this.column(columnKey)))) {
                        Map unused = StandardTable.this.removeColumn(columnKey);
                        changed = true;
                    }
                }
                return changed;
            }
        }

        private class ColumnMapValues extends StandardTable<R, C, V>.TableCollection<Map<R, V>> {
            private ColumnMapValues() {
                super();
            }

            public Iterator<Map<R, V>> iterator() {
                return Maps.valueIterator(ColumnMap.this.entrySet().iterator());
            }

            public boolean remove(Object obj) {
                for (Map.Entry<C, Map<R, V>> entry : ColumnMap.this.entrySet()) {
                    if (entry.getValue().equals(obj)) {
                        Map unused = StandardTable.this.removeColumn(entry.getKey());
                        return true;
                    }
                }
                return false;
            }

            public boolean removeAll(Collection<?> c) {
                Preconditions.checkNotNull(c);
                boolean changed = false;
                Iterator i$ = Lists.newArrayList(StandardTable.this.columnKeySet().iterator()).iterator();
                while (i$.hasNext()) {
                    C columnKey = i$.next();
                    if (c.contains(StandardTable.this.column(columnKey))) {
                        Map unused = StandardTable.this.removeColumn(columnKey);
                        changed = true;
                    }
                }
                return changed;
            }

            public boolean retainAll(Collection<?> c) {
                Preconditions.checkNotNull(c);
                boolean changed = false;
                Iterator i$ = Lists.newArrayList(StandardTable.this.columnKeySet().iterator()).iterator();
                while (i$.hasNext()) {
                    C columnKey = i$.next();
                    if (!c.contains(StandardTable.this.column(columnKey))) {
                        Map unused = StandardTable.this.removeColumn(columnKey);
                        changed = true;
                    }
                }
                return changed;
            }

            public int size() {
                return StandardTable.this.columnKeySet().size();
            }
        }
    }
}
