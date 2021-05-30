package com.google.common.collect;

import com.google.common.annotations.GwtCompatible;
import com.google.common.annotations.GwtIncompatible;
import com.google.common.base.Objects;
import com.google.common.base.Preconditions;
import com.google.common.collect.Multimaps;
import com.google.common.collect.Multiset;
import com.google.common.collect.Multisets;
import java.io.IOException;
import java.io.ObjectInputStream;
import java.io.ObjectOutputStream;
import java.io.Serializable;
import java.util.AbstractSequentialList;
import java.util.AbstractSet;
import java.util.Collection;
import java.util.Collections;
import java.util.Iterator;
import java.util.List;
import java.util.ListIterator;
import java.util.Map;
import java.util.NoSuchElementException;
import java.util.Set;
import javax.annotation.Nullable;

@GwtCompatible(emulated = true, serializable = true)
public class LinkedListMultimap<K, V> implements ListMultimap<K, V>, Serializable {
    @GwtIncompatible("java serialization not supported")
    private static final long serialVersionUID = 0;
    private transient List<Map.Entry<K, V>> entries;
    /* access modifiers changed from: private */
    public transient Node<K, V> head;
    /* access modifiers changed from: private */
    public transient Multiset<K> keyCount;
    private transient Set<K> keySet;
    /* access modifiers changed from: private */
    public transient Map<K, Node<K, V>> keyToKeyHead;
    /* access modifiers changed from: private */
    public transient Map<K, Node<K, V>> keyToKeyTail;
    private transient Multiset<K> keys;
    private transient Map<K, Collection<V>> map;
    /* access modifiers changed from: private */
    public transient Node<K, V> tail;
    private transient List<V> valuesList;

    private static final class Node<K, V> {
        final K key;
        Node<K, V> next;
        Node<K, V> nextSibling;
        Node<K, V> previous;
        Node<K, V> previousSibling;
        V value;

        Node(@Nullable K key2, @Nullable V value2) {
            this.key = key2;
            this.value = value2;
        }

        public String toString() {
            return this.key + "=" + this.value;
        }
    }

    public static <K, V> LinkedListMultimap<K, V> create() {
        return new LinkedListMultimap<>();
    }

    public static <K, V> LinkedListMultimap<K, V> create(int expectedKeys) {
        return new LinkedListMultimap<>(expectedKeys);
    }

    public static <K, V> LinkedListMultimap<K, V> create(Multimap<? extends K, ? extends V> multimap) {
        return new LinkedListMultimap<>(multimap);
    }

    LinkedListMultimap() {
        this.keyCount = LinkedHashMultiset.create();
        this.keyToKeyHead = Maps.newHashMap();
        this.keyToKeyTail = Maps.newHashMap();
    }

    private LinkedListMultimap(int expectedKeys) {
        this.keyCount = LinkedHashMultiset.create(expectedKeys);
        this.keyToKeyHead = Maps.newHashMapWithExpectedSize(expectedKeys);
        this.keyToKeyTail = Maps.newHashMapWithExpectedSize(expectedKeys);
    }

    private LinkedListMultimap(Multimap<? extends K, ? extends V> multimap) {
        this(multimap.keySet().size());
        putAll(multimap);
    }

    /* access modifiers changed from: private */
    public Node<K, V> addNode(@Nullable K key, @Nullable V value, @Nullable Node<K, V> nextSibling) {
        Node<K, V> node = new Node<>(key, value);
        if (this.head == null) {
            this.tail = node;
            this.head = node;
            this.keyToKeyHead.put(key, node);
            this.keyToKeyTail.put(key, node);
        } else if (nextSibling == null) {
            this.tail.next = node;
            node.previous = this.tail;
            Node<K, V> keyTail = this.keyToKeyTail.get(key);
            if (keyTail == null) {
                this.keyToKeyHead.put(key, node);
            } else {
                keyTail.nextSibling = node;
                node.previousSibling = keyTail;
            }
            this.keyToKeyTail.put(key, node);
            this.tail = node;
        } else {
            node.previous = nextSibling.previous;
            node.previousSibling = nextSibling.previousSibling;
            node.next = nextSibling;
            node.nextSibling = nextSibling;
            if (nextSibling.previousSibling == null) {
                this.keyToKeyHead.put(key, node);
            } else {
                nextSibling.previousSibling.nextSibling = node;
            }
            if (nextSibling.previous == null) {
                this.head = node;
            } else {
                nextSibling.previous.next = node;
            }
            nextSibling.previous = node;
            nextSibling.previousSibling = node;
        }
        this.keyCount.add(key);
        return node;
    }

    /* access modifiers changed from: private */
    public void removeNode(Node<K, V> node) {
        if (node.previous != null) {
            node.previous.next = node.next;
        } else {
            this.head = node.next;
        }
        if (node.next != null) {
            node.next.previous = node.previous;
        } else {
            this.tail = node.previous;
        }
        if (node.previousSibling != null) {
            node.previousSibling.nextSibling = node.nextSibling;
        } else if (node.nextSibling != null) {
            this.keyToKeyHead.put(node.key, node.nextSibling);
        } else {
            this.keyToKeyHead.remove(node.key);
        }
        if (node.nextSibling != null) {
            node.nextSibling.previousSibling = node.previousSibling;
        } else if (node.previousSibling != null) {
            this.keyToKeyTail.put(node.key, node.previousSibling);
        } else {
            this.keyToKeyTail.remove(node.key);
        }
        this.keyCount.remove(node.key);
    }

    /* access modifiers changed from: private */
    public void removeAllNodes(@Nullable Object key) {
        Iterator<V> i = new ValueForKeyIterator(key);
        while (i.hasNext()) {
            i.next();
            i.remove();
        }
    }

    /* access modifiers changed from: private */
    public static void checkElement(@Nullable Object node) {
        if (node == null) {
            throw new NoSuchElementException();
        }
    }

    private class NodeIterator implements ListIterator<Node<K, V>> {
        Node<K, V> current;
        Node<K, V> next;
        int nextIndex;
        Node<K, V> previous;

        NodeIterator() {
            this.next = LinkedListMultimap.this.head;
        }

        NodeIterator(int index) {
            int size = LinkedListMultimap.this.size();
            Preconditions.checkPositionIndex(index, size);
            if (index < size / 2) {
                this.next = LinkedListMultimap.this.head;
                while (true) {
                    int index2 = index - 1;
                    if (index <= 0) {
                        break;
                    }
                    next();
                    index = index2;
                }
            } else {
                this.previous = LinkedListMultimap.this.tail;
                this.nextIndex = size;
                while (true) {
                    int index3 = index + 1;
                    if (index >= size) {
                        break;
                    }
                    previous();
                    index = index3;
                }
            }
            this.current = null;
        }

        public boolean hasNext() {
            return this.next != null;
        }

        public Node<K, V> next() {
            LinkedListMultimap.checkElement(this.next);
            Node<K, V> node = this.next;
            this.current = node;
            this.previous = node;
            this.next = this.next.next;
            this.nextIndex++;
            return this.current;
        }

        public void remove() {
            Preconditions.checkState(this.current != null);
            if (this.current != this.next) {
                this.previous = this.current.previous;
                this.nextIndex--;
            } else {
                this.next = this.current.next;
            }
            LinkedListMultimap.this.removeNode(this.current);
            this.current = null;
        }

        public boolean hasPrevious() {
            return this.previous != null;
        }

        public Node<K, V> previous() {
            LinkedListMultimap.checkElement(this.previous);
            Node<K, V> node = this.previous;
            this.current = node;
            this.next = node;
            this.previous = this.previous.previous;
            this.nextIndex--;
            return this.current;
        }

        public int nextIndex() {
            return this.nextIndex;
        }

        public int previousIndex() {
            return this.nextIndex - 1;
        }

        public void set(Node<K, V> node) {
            throw new UnsupportedOperationException();
        }

        public void add(Node<K, V> node) {
            throw new UnsupportedOperationException();
        }

        /* access modifiers changed from: package-private */
        public void setValue(V value) {
            Preconditions.checkState(this.current != null);
            this.current.value = value;
        }
    }

    private class DistinctKeyIterator implements Iterator<K> {
        Node<K, V> current;
        Node<K, V> next;
        final Set<K> seenKeys;

        private DistinctKeyIterator() {
            this.seenKeys = Sets.newHashSetWithExpectedSize(LinkedListMultimap.this.keySet().size());
            this.next = LinkedListMultimap.this.head;
        }

        public boolean hasNext() {
            return this.next != null;
        }

        public K next() {
            LinkedListMultimap.checkElement(this.next);
            this.current = this.next;
            this.seenKeys.add(this.current.key);
            do {
                this.next = this.next.next;
                if (this.next == null || this.seenKeys.add(this.next.key)) {
                }
                this.next = this.next.next;
                break;
            } while (this.seenKeys.add(this.next.key));
            return this.current.key;
        }

        public void remove() {
            Preconditions.checkState(this.current != null);
            LinkedListMultimap.this.removeAllNodes(this.current.key);
            this.current = null;
        }
    }

    private class ValueForKeyIterator implements ListIterator<V> {
        Node<K, V> current;
        final Object key;
        Node<K, V> next;
        int nextIndex;
        Node<K, V> previous;

        ValueForKeyIterator(@Nullable Object key2) {
            this.key = key2;
            this.next = (Node) LinkedListMultimap.this.keyToKeyHead.get(key2);
        }

        public ValueForKeyIterator(@Nullable Object key2, int index) {
            int size = LinkedListMultimap.this.keyCount.count(key2);
            Preconditions.checkPositionIndex(index, size);
            if (index < size / 2) {
                this.next = (Node) LinkedListMultimap.this.keyToKeyHead.get(key2);
                while (true) {
                    int index2 = index - 1;
                    if (index <= 0) {
                        break;
                    }
                    next();
                    index = index2;
                }
            } else {
                this.previous = (Node) LinkedListMultimap.this.keyToKeyTail.get(key2);
                this.nextIndex = size;
                while (true) {
                    int index3 = index + 1;
                    if (index >= size) {
                        break;
                    }
                    previous();
                    index = index3;
                }
            }
            this.key = key2;
            this.current = null;
        }

        public boolean hasNext() {
            return this.next != null;
        }

        public V next() {
            LinkedListMultimap.checkElement(this.next);
            Node<K, V> node = this.next;
            this.current = node;
            this.previous = node;
            this.next = this.next.nextSibling;
            this.nextIndex++;
            return this.current.value;
        }

        public boolean hasPrevious() {
            return this.previous != null;
        }

        public V previous() {
            LinkedListMultimap.checkElement(this.previous);
            Node<K, V> node = this.previous;
            this.current = node;
            this.next = node;
            this.previous = this.previous.previousSibling;
            this.nextIndex--;
            return this.current.value;
        }

        public int nextIndex() {
            return this.nextIndex;
        }

        public int previousIndex() {
            return this.nextIndex - 1;
        }

        public void remove() {
            Preconditions.checkState(this.current != null);
            if (this.current != this.next) {
                this.previous = this.current.previousSibling;
                this.nextIndex--;
            } else {
                this.next = this.current.nextSibling;
            }
            LinkedListMultimap.this.removeNode(this.current);
            this.current = null;
        }

        public void set(V value) {
            Preconditions.checkState(this.current != null);
            this.current.value = value;
        }

        public void add(V value) {
            this.previous = LinkedListMultimap.this.addNode(this.key, value, this.next);
            this.nextIndex++;
            this.current = null;
        }
    }

    public int size() {
        return this.keyCount.size();
    }

    public boolean isEmpty() {
        return this.head == null;
    }

    public boolean containsKey(@Nullable Object key) {
        return this.keyToKeyHead.containsKey(key);
    }

    public boolean containsValue(@Nullable Object value) {
        Iterator<Node<K, V>> i = new NodeIterator();
        while (i.hasNext()) {
            if (Objects.equal(i.next().value, value)) {
                return true;
            }
        }
        return false;
    }

    public boolean containsEntry(@Nullable Object key, @Nullable Object value) {
        Iterator<V> i = new ValueForKeyIterator(key);
        while (i.hasNext()) {
            if (Objects.equal(i.next(), value)) {
                return true;
            }
        }
        return false;
    }

    public boolean put(@Nullable K key, @Nullable V value) {
        addNode(key, value, (Node) null);
        return true;
    }

    public boolean remove(@Nullable Object key, @Nullable Object value) {
        Iterator<V> values = new ValueForKeyIterator(key);
        while (values.hasNext()) {
            if (Objects.equal(values.next(), value)) {
                values.remove();
                return true;
            }
        }
        return false;
    }

    public boolean putAll(@Nullable K key, Iterable<? extends V> values) {
        boolean changed = false;
        for (V value : values) {
            changed |= put(key, value);
        }
        return changed;
    }

    public boolean putAll(Multimap<? extends K, ? extends V> multimap) {
        boolean changed = false;
        for (Map.Entry<? extends K, ? extends V> entry : multimap.entries()) {
            changed |= put(entry.getKey(), entry.getValue());
        }
        return changed;
    }

    public List<V> replaceValues(@Nullable K key, Iterable<? extends V> values) {
        List<V> oldValues = getCopy(key);
        ListIterator<V> keyValues = new ValueForKeyIterator(key);
        Iterator<? extends V> newValues = values.iterator();
        while (keyValues.hasNext() && newValues.hasNext()) {
            keyValues.next();
            keyValues.set(newValues.next());
        }
        while (keyValues.hasNext()) {
            keyValues.next();
            keyValues.remove();
        }
        while (newValues.hasNext()) {
            keyValues.add(newValues.next());
        }
        return oldValues;
    }

    private List<V> getCopy(@Nullable Object key) {
        return Collections.unmodifiableList(Lists.newArrayList(new ValueForKeyIterator(key)));
    }

    public List<V> removeAll(@Nullable Object key) {
        List<V> oldValues = getCopy(key);
        removeAllNodes(key);
        return oldValues;
    }

    public void clear() {
        this.head = null;
        this.tail = null;
        this.keyCount.clear();
        this.keyToKeyHead.clear();
        this.keyToKeyTail.clear();
    }

    public List<V> get(@Nullable final K key) {
        return new AbstractSequentialList<V>() {
            public int size() {
                return LinkedListMultimap.this.keyCount.count(key);
            }

            public ListIterator<V> listIterator(int index) {
                return new ValueForKeyIterator(key, index);
            }

            public boolean removeAll(Collection<?> c) {
                return Iterators.removeAll(iterator(), c);
            }

            public boolean retainAll(Collection<?> c) {
                return Iterators.retainAll(iterator(), c);
            }
        };
    }

    public Set<K> keySet() {
        Set<K> result = this.keySet;
        if (result != null) {
            return result;
        }
        Set<K> r1 = new AbstractSet<K>() {
            public int size() {
                return LinkedListMultimap.this.keyCount.elementSet().size();
            }

            public Iterator<K> iterator() {
                return new DistinctKeyIterator();
            }

            public boolean contains(Object key) {
                return LinkedListMultimap.this.containsKey(key);
            }

            public boolean remove(Object o) {
                return !LinkedListMultimap.this.removeAll(o).isEmpty();
            }

            public boolean removeAll(Collection<?> c) {
                Preconditions.checkNotNull(c);
                return super.removeAll(c);
            }
        };
        Set<K> result2 = r1;
        this.keySet = r1;
        return result2;
    }

    public Multiset<K> keys() {
        Multiset<K> result = this.keys;
        if (result != null) {
            return result;
        }
        Multiset<K> multisetView = new MultisetView();
        Multiset<K> result2 = multisetView;
        this.keys = multisetView;
        return result2;
    }

    private class MultisetView extends AbstractMultiset<K> {
        private MultisetView() {
        }

        public int size() {
            return LinkedListMultimap.this.keyCount.size();
        }

        public int count(Object element) {
            return LinkedListMultimap.this.keyCount.count(element);
        }

        /* access modifiers changed from: package-private */
        public Iterator<Multiset.Entry<K>> entryIterator() {
            return new TransformedIterator<K, Multiset.Entry<K>>(new DistinctKeyIterator()) {
                /* access modifiers changed from: package-private */
                public Multiset.Entry<K> transform(final K key) {
                    return new Multisets.AbstractEntry<K>() {
                        public K getElement() {
                            return key;
                        }

                        public int getCount() {
                            return LinkedListMultimap.this.keyCount.count(key);
                        }
                    };
                }
            };
        }

        /* access modifiers changed from: package-private */
        public int distinctElements() {
            return elementSet().size();
        }

        public Iterator<K> iterator() {
            return new TransformedIterator<Node<K, V>, K>(new NodeIterator()) {
                /* access modifiers changed from: package-private */
                public K transform(Node<K, V> node) {
                    return node.key;
                }
            };
        }

        public int remove(@Nullable Object key, int occurrences) {
            Preconditions.checkArgument(occurrences >= 0);
            int oldCount = count(key);
            Iterator<V> values = new ValueForKeyIterator(key);
            while (true) {
                int occurrences2 = occurrences - 1;
                if (occurrences <= 0 || values.hasNext() == 0) {
                    return oldCount;
                }
                values.next();
                values.remove();
                occurrences = occurrences2;
            }
            return oldCount;
        }

        public Set<K> elementSet() {
            return LinkedListMultimap.this.keySet();
        }

        public boolean equals(@Nullable Object object) {
            return LinkedListMultimap.this.keyCount.equals(object);
        }

        public int hashCode() {
            return LinkedListMultimap.this.keyCount.hashCode();
        }

        public String toString() {
            return LinkedListMultimap.this.keyCount.toString();
        }
    }

    public List<V> values() {
        List<V> result = this.valuesList;
        if (result != null) {
            return result;
        }
        List<V> r1 = new AbstractSequentialList<V>() {
            public int size() {
                return LinkedListMultimap.this.keyCount.size();
            }

            public ListIterator<V> listIterator(int index) {
                final LinkedListMultimap<K, V>.NodeIterator nodes = new NodeIterator(index);
                return new TransformedListIterator<Node<K, V>, V>(nodes) {
                    /* access modifiers changed from: package-private */
                    public V transform(Node<K, V> node) {
                        return node.value;
                    }

                    public void set(V value) {
                        nodes.setValue(value);
                    }
                };
            }
        };
        List<V> result2 = r1;
        this.valuesList = r1;
        return result2;
    }

    /* access modifiers changed from: private */
    public static <K, V> Map.Entry<K, V> createEntry(final Node<K, V> node) {
        return new AbstractMapEntry<K, V>() {
            public K getKey() {
                return node.key;
            }

            public V getValue() {
                return node.value;
            }

            public V setValue(V value) {
                V oldValue = node.value;
                node.value = value;
                return oldValue;
            }
        };
    }

    public List<Map.Entry<K, V>> entries() {
        List<Map.Entry<K, V>> result = this.entries;
        if (result != null) {
            return result;
        }
        List<Map.Entry<K, V>> r1 = new AbstractSequentialList<Map.Entry<K, V>>() {
            public int size() {
                return LinkedListMultimap.this.keyCount.size();
            }

            public ListIterator<Map.Entry<K, V>> listIterator(int index) {
                return new TransformedListIterator<Node<K, V>, Map.Entry<K, V>>(new NodeIterator(index)) {
                    /* access modifiers changed from: package-private */
                    public Map.Entry<K, V> transform(Node<K, V> node) {
                        return LinkedListMultimap.createEntry(node);
                    }
                };
            }
        };
        List<Map.Entry<K, V>> result2 = r1;
        this.entries = r1;
        return result2;
    }

    public Map<K, Collection<V>> asMap() {
        Map<K, Collection<V>> result = this.map;
        if (result != null) {
            return result;
        }
        Map<K, Collection<V>> r1 = new Multimaps.AsMap<K, V>() {
            public int size() {
                return LinkedListMultimap.this.keyCount.elementSet().size();
            }

            /* access modifiers changed from: package-private */
            public Multimap<K, V> multimap() {
                return LinkedListMultimap.this;
            }

            /* access modifiers changed from: package-private */
            public Iterator<Map.Entry<K, Collection<V>>> entryIterator() {
                return new TransformedIterator<K, Map.Entry<K, Collection<V>>>(new DistinctKeyIterator()) {
                    /* access modifiers changed from: package-private */
                    public Map.Entry<K, Collection<V>> transform(final K key) {
                        return new AbstractMapEntry<K, Collection<V>>() {
                            public K getKey() {
                                return key;
                            }

                            public Collection<V> getValue() {
                                return LinkedListMultimap.this.get(key);
                            }
                        };
                    }
                };
            }
        };
        Map<K, Collection<V>> result2 = r1;
        this.map = r1;
        return result2;
    }

    public boolean equals(@Nullable Object other) {
        if (other == this) {
            return true;
        }
        if (other instanceof Multimap) {
            return asMap().equals(((Multimap) other).asMap());
        }
        return false;
    }

    public int hashCode() {
        return asMap().hashCode();
    }

    public String toString() {
        return asMap().toString();
    }

    @GwtIncompatible("java.io.ObjectOutputStream")
    private void writeObject(ObjectOutputStream stream) throws IOException {
        stream.defaultWriteObject();
        stream.writeInt(size());
        for (Map.Entry<K, V> entry : entries()) {
            stream.writeObject(entry.getKey());
            stream.writeObject(entry.getValue());
        }
    }

    @GwtIncompatible("java.io.ObjectInputStream")
    private void readObject(ObjectInputStream stream) throws IOException, ClassNotFoundException {
        stream.defaultReadObject();
        this.keyCount = LinkedHashMultiset.create();
        this.keyToKeyHead = Maps.newHashMap();
        this.keyToKeyTail = Maps.newHashMap();
        int size = stream.readInt();
        for (int i = 0; i < size; i++) {
            put(stream.readObject(), stream.readObject());
        }
    }
}
