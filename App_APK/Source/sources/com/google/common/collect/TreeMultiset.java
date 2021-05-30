package com.google.common.collect;

import com.google.common.annotations.GwtCompatible;
import com.google.common.annotations.GwtIncompatible;
import com.google.common.base.Optional;
import com.google.common.base.Preconditions;
import com.google.common.collect.Multiset;
import com.google.common.collect.Multisets;
import com.google.common.primitives.Ints;
import java.io.IOException;
import java.io.ObjectInputStream;
import java.io.ObjectOutputStream;
import java.io.Serializable;
import java.util.Collection;
import java.util.Comparator;
import java.util.ConcurrentModificationException;
import java.util.Iterator;
import java.util.Set;
import java.util.SortedSet;
import javax.annotation.Nullable;

@GwtCompatible(emulated = true)
public final class TreeMultiset<E> extends AbstractSortedMultiset<E> implements Serializable {
    private static final BstAggregate<Node<Object>> DISTINCT_AGGREGATE = new BstAggregate<Node<Object>>() {
        public int entryValue(Node<Object> node) {
            return 1;
        }

        public long treeValue(@Nullable Node<Object> tree) {
            return (long) TreeMultiset.distinctOrZero(tree);
        }
    };
    private static final BstNodeFactory<Node<Object>> NODE_FACTORY = new BstNodeFactory<Node<Object>>() {
        public Node<Object> createNode(Node<Object> source, @Nullable Node<Object> left, @Nullable Node<Object> right) {
            return new Node(source.getKey(), source.elemCount(), left, right);
        }
    };
    private static final BstAggregate<Node<Object>> SIZE_AGGREGATE = new BstAggregate<Node<Object>>() {
        public int entryValue(Node<Object> entry) {
            return entry.elemCount();
        }

        public long treeValue(@Nullable Node<Object> tree) {
            return TreeMultiset.sizeOrZero(tree);
        }
    };
    @GwtIncompatible("not needed in emulated source")
    private static final long serialVersionUID = 1;
    /* access modifiers changed from: private */
    public final transient GeneralRange<E> range;
    /* access modifiers changed from: private */
    public final transient Reference<Node<E>> rootReference;

    public /* bridge */ /* synthetic */ boolean add(Object x0) {
        return super.add(x0);
    }

    public /* bridge */ /* synthetic */ boolean addAll(Collection x0) {
        return super.addAll(x0);
    }

    public /* bridge */ /* synthetic */ boolean contains(Object x0) {
        return super.contains(x0);
    }

    public /* bridge */ /* synthetic */ SortedMultiset descendingMultiset() {
        return super.descendingMultiset();
    }

    public /* bridge */ /* synthetic */ SortedSet elementSet() {
        return super.elementSet();
    }

    public /* bridge */ /* synthetic */ Set entrySet() {
        return super.entrySet();
    }

    public /* bridge */ /* synthetic */ boolean equals(Object x0) {
        return super.equals(x0);
    }

    public /* bridge */ /* synthetic */ Multiset.Entry firstEntry() {
        return super.firstEntry();
    }

    public /* bridge */ /* synthetic */ int hashCode() {
        return super.hashCode();
    }

    public /* bridge */ /* synthetic */ boolean isEmpty() {
        return super.isEmpty();
    }

    public /* bridge */ /* synthetic */ Multiset.Entry lastEntry() {
        return super.lastEntry();
    }

    public /* bridge */ /* synthetic */ Multiset.Entry pollFirstEntry() {
        return super.pollFirstEntry();
    }

    public /* bridge */ /* synthetic */ Multiset.Entry pollLastEntry() {
        return super.pollLastEntry();
    }

    public /* bridge */ /* synthetic */ boolean remove(Object x0) {
        return super.remove(x0);
    }

    public /* bridge */ /* synthetic */ boolean removeAll(Collection x0) {
        return super.removeAll(x0);
    }

    public /* bridge */ /* synthetic */ boolean retainAll(Collection x0) {
        return super.retainAll(x0);
    }

    public /* bridge */ /* synthetic */ SortedMultiset subMultiset(Object x0, BoundType x1, Object x2, BoundType x3) {
        return super.subMultiset(x0, x1, x2, x3);
    }

    public /* bridge */ /* synthetic */ String toString() {
        return super.toString();
    }

    public static <E extends Comparable> TreeMultiset<E> create() {
        return new TreeMultiset<>(Ordering.natural());
    }

    public static <E> TreeMultiset<E> create(@Nullable Comparator<? super E> comparator) {
        return comparator == null ? new TreeMultiset<>(Ordering.natural()) : new TreeMultiset<>(comparator);
    }

    public static <E extends Comparable> TreeMultiset<E> create(Iterable<? extends E> elements) {
        TreeMultiset<E> multiset = create();
        Iterables.addAll(multiset, elements);
        return multiset;
    }

    public Iterator<E> iterator() {
        return super.iterator();
    }

    private TreeMultiset(Comparator<? super E> comparator) {
        super(comparator);
        this.range = GeneralRange.all(comparator);
        this.rootReference = new Reference<>();
    }

    private TreeMultiset(GeneralRange<E> range2, Reference<Node<E>> root) {
        super(range2.comparator());
        this.range = range2;
        this.rootReference = root;
    }

    /* access modifiers changed from: package-private */
    public E checkElement(Object o) {
        Object obj = o;
        this.comparator.compare(obj, obj);
        return obj;
    }

    static final class Reference<T> {
        T value;

        public T get() {
            return this.value;
        }

        public boolean compareAndSet(T expected, T newValue) {
            if (this.value != expected) {
                return false;
            }
            this.value = newValue;
            return true;
        }
    }

    /* access modifiers changed from: package-private */
    public int distinctElements() {
        return Ints.checkedCast(BstRangeOps.totalInRange(distinctAggregate(), this.range, this.rootReference.get()));
    }

    public int size() {
        return Ints.saturatedCast(BstRangeOps.totalInRange(sizeAggregate(), this.range, this.rootReference.get()));
    }

    public int count(@Nullable Object element) {
        try {
            E e = checkElement(element);
            if (this.range.contains(e)) {
                return countOrZero((Node) BstOperations.seek(comparator(), this.rootReference.get(), e));
            }
            return 0;
        } catch (ClassCastException e2) {
            return 0;
        } catch (NullPointerException e3) {
            return 0;
        }
    }

    private int mutate(@Nullable E e, TreeMultiset<E>.MultisetModifier modifier) {
        BstMutationResult<E, Node<E>> mutationResult = BstOperations.mutate(comparator(), BstMutationRule.createRule(modifier, BstCountBasedBalancePolicies.singleRebalancePolicy(distinctAggregate()), nodeFactory()), this.rootReference.get(), e);
        if (this.rootReference.compareAndSet(mutationResult.getOriginalRoot(), mutationResult.getChangedRoot())) {
            return countOrZero(mutationResult.getOriginalTarget());
        }
        throw new ConcurrentModificationException();
    }

    public int add(E element, int occurrences) {
        checkElement(element);
        if (occurrences == 0) {
            return count(element);
        }
        Preconditions.checkArgument(this.range.contains(element));
        return mutate(element, new AddModifier(occurrences));
    }

    public int remove(@Nullable Object element, int occurrences) {
        if (occurrences == 0) {
            return count(element);
        }
        try {
            E e = checkElement(element);
            if (this.range.contains(e)) {
                return mutate(e, new RemoveModifier(occurrences));
            }
            return 0;
        } catch (ClassCastException e2) {
            return 0;
        } catch (NullPointerException e3) {
            return 0;
        }
    }

    public boolean setCount(E element, int oldCount, int newCount) {
        checkElement(element);
        Preconditions.checkArgument(this.range.contains(element));
        return mutate(element, new ConditionalSetCountModifier(oldCount, newCount)) == oldCount;
    }

    public int setCount(E element, int count) {
        checkElement(element);
        Preconditions.checkArgument(this.range.contains(element));
        return mutate(element, new SetCountModifier(count));
    }

    private BstPathFactory<Node<E>, BstInOrderPath<Node<E>>> pathFactory() {
        return BstInOrderPath.inOrderFactory();
    }

    /* access modifiers changed from: package-private */
    public Iterator<Multiset.Entry<E>> entryIterator() {
        return iteratorInDirection((BstInOrderPath) BstRangeOps.furthestPath(this.range, BstSide.LEFT, pathFactory(), this.rootReference.get()), BstSide.RIGHT);
    }

    /* access modifiers changed from: package-private */
    public Iterator<Multiset.Entry<E>> descendingEntryIterator() {
        return iteratorInDirection((BstInOrderPath) BstRangeOps.furthestPath(this.range, BstSide.RIGHT, pathFactory(), this.rootReference.get()), BstSide.LEFT);
    }

    private Iterator<Multiset.Entry<E>> iteratorInDirection(@Nullable BstInOrderPath<Node<E>> start, final BstSide direction) {
        final Iterator<BstInOrderPath<Node<E>>> pathIterator = new AbstractSequentialIterator<BstInOrderPath<Node<E>>>(start) {
            /* access modifiers changed from: protected */
            public BstInOrderPath<Node<E>> computeNext(BstInOrderPath<Node<E>> previous) {
                if (!previous.hasNext(direction)) {
                    return null;
                }
                BstInOrderPath<Node<E>> next = previous.next(direction);
                if (TreeMultiset.this.range.contains(next.getTip().getKey())) {
                    return next;
                }
                return null;
            }
        };
        return new Iterator<Multiset.Entry<E>>() {
            final ToRemove<E> toRemove = new ToRemove<>();

            public boolean hasNext() {
                return pathIterator.hasNext();
            }

            public Multiset.Entry<E> next() {
                BstInOrderPath<Node<E>> path = (BstInOrderPath) pathIterator.next();
                return new LiveEntry(this.toRemove.setAndGet(path.getTip().getKey()), path.getTip().elemCount());
            }

            public void remove() {
                TreeMultiset.this.setCount(this.toRemove.getAndClear(), 0);
            }
        };
    }

    private static final class ToRemove<E> {
        @Nullable
        Optional<E> element;

        private ToRemove() {
        }

        /* access modifiers changed from: package-private */
        public E setAndGet(@Nullable E element2) {
            this.element = Optional.fromNullable(element2);
            return element2;
        }

        /* access modifiers changed from: package-private */
        public E getAndClear() {
            Preconditions.checkState(this.element != null);
            E returnValue = this.element.orNull();
            this.element = null;
            return returnValue;
        }
    }

    class LiveEntry extends Multisets.AbstractEntry<E> {
        private int count;
        private final E element;
        private Node<E> expectedRoot;

        private LiveEntry(E element2, int count2) {
            this.expectedRoot = (Node) TreeMultiset.this.rootReference.get();
            this.element = element2;
            this.count = count2;
        }

        public E getElement() {
            return this.element;
        }

        public int getCount() {
            if (TreeMultiset.this.rootReference.get() == this.expectedRoot) {
                return this.count;
            }
            this.expectedRoot = (Node) TreeMultiset.this.rootReference.get();
            int count2 = TreeMultiset.this.count(this.element);
            this.count = count2;
            return count2;
        }
    }

    public void clear() {
        Node<E> root = this.rootReference.get();
        if (!this.rootReference.compareAndSet(root, (Node) BstRangeOps.minusRange(this.range, BstCountBasedBalancePolicies.fullRebalancePolicy(distinctAggregate()), nodeFactory(), root))) {
            throw new ConcurrentModificationException();
        }
    }

    public SortedMultiset<E> headMultiset(E upperBound, BoundType boundType) {
        Preconditions.checkNotNull(upperBound);
        return new TreeMultiset(this.range.intersect(GeneralRange.upTo(this.comparator, upperBound, boundType)), this.rootReference);
    }

    public SortedMultiset<E> tailMultiset(E lowerBound, BoundType boundType) {
        Preconditions.checkNotNull(lowerBound);
        return new TreeMultiset(this.range.intersect(GeneralRange.downTo(this.comparator, lowerBound, boundType)), this.rootReference);
    }

    public Comparator<? super E> comparator() {
        return super.comparator();
    }

    private static final class Node<E> extends BstNode<E, Node<E>> implements Serializable {
        private static final long serialVersionUID = 0;
        /* access modifiers changed from: private */
        public final int distinct;
        /* access modifiers changed from: private */
        public final long size;

        private Node(E key, int elemCount, @Nullable Node<E> left, @Nullable Node<E> right) {
            super(key, left, right);
            Preconditions.checkArgument(elemCount > 0);
            this.size = ((long) elemCount) + TreeMultiset.sizeOrZero(left) + TreeMultiset.sizeOrZero(right);
            this.distinct = TreeMultiset.distinctOrZero(left) + 1 + TreeMultiset.distinctOrZero(right);
        }

        /* access modifiers changed from: package-private */
        public int elemCount() {
            return Ints.checkedCast((this.size - TreeMultiset.sizeOrZero((Node) childOrNull(BstSide.LEFT))) - TreeMultiset.sizeOrZero((Node) childOrNull(BstSide.RIGHT)));
        }

        private Node(E key, int elemCount) {
            this(key, elemCount, (Node) null, (Node) null);
        }
    }

    /* access modifiers changed from: private */
    public static long sizeOrZero(@Nullable Node<?> node) {
        if (node == null) {
            return 0;
        }
        return node.size;
    }

    /* access modifiers changed from: private */
    public static int distinctOrZero(@Nullable Node<?> node) {
        if (node == null) {
            return 0;
        }
        return node.distinct;
    }

    /* access modifiers changed from: private */
    public static int countOrZero(@Nullable Node<?> entry) {
        if (entry == null) {
            return 0;
        }
        return entry.elemCount();
    }

    private BstAggregate<Node<E>> distinctAggregate() {
        return DISTINCT_AGGREGATE;
    }

    private BstAggregate<Node<E>> sizeAggregate() {
        return SIZE_AGGREGATE;
    }

    private BstNodeFactory<Node<E>> nodeFactory() {
        return NODE_FACTORY;
    }

    private abstract class MultisetModifier implements BstModifier<E, Node<E>> {
        /* access modifiers changed from: package-private */
        public abstract int newCount(int i);

        private MultisetModifier() {
        }

        @Nullable
        public BstModificationResult<Node<E>> modify(E key, @Nullable Node<E> originalEntry) {
            int oldCount = TreeMultiset.countOrZero(originalEntry);
            int newCount = newCount(oldCount);
            if (oldCount == newCount) {
                return BstModificationResult.identity(originalEntry);
            }
            if (newCount == 0) {
                return BstModificationResult.rebalancingChange(originalEntry, null);
            }
            if (oldCount == 0) {
                return BstModificationResult.rebalancingChange(null, new Node(key, newCount));
            }
            return BstModificationResult.rebuildingChange(originalEntry, new Node(originalEntry.getKey(), newCount));
        }
    }

    private final class AddModifier extends MultisetModifier {
        private final int countToAdd;

        private AddModifier(int countToAdd2) {
            super();
            Preconditions.checkArgument(countToAdd2 > 0);
            this.countToAdd = countToAdd2;
        }

        /* access modifiers changed from: package-private */
        public int newCount(int oldCount) {
            Preconditions.checkArgument(this.countToAdd <= Integer.MAX_VALUE - oldCount, "Cannot add this many elements");
            return this.countToAdd + oldCount;
        }
    }

    private final class RemoveModifier extends MultisetModifier {
        private final int countToRemove;

        private RemoveModifier(int countToRemove2) {
            super();
            Preconditions.checkArgument(countToRemove2 > 0);
            this.countToRemove = countToRemove2;
        }

        /* access modifiers changed from: package-private */
        public int newCount(int oldCount) {
            return Math.max(0, oldCount - this.countToRemove);
        }
    }

    private final class SetCountModifier extends MultisetModifier {
        private final int countToSet;

        private SetCountModifier(int countToSet2) {
            super();
            Preconditions.checkArgument(countToSet2 >= 0);
            this.countToSet = countToSet2;
        }

        /* access modifiers changed from: package-private */
        public int newCount(int oldCount) {
            return this.countToSet;
        }
    }

    private final class ConditionalSetCountModifier extends MultisetModifier {
        private final int expectedCount;
        private final int setCount;

        private ConditionalSetCountModifier(int expectedCount2, int setCount2) {
            super();
            boolean z = false;
            Preconditions.checkArgument((expectedCount2 >= 0 ? true : z) & (setCount2 >= 0));
            this.expectedCount = expectedCount2;
            this.setCount = setCount2;
        }

        /* access modifiers changed from: package-private */
        public int newCount(int oldCount) {
            return oldCount == this.expectedCount ? this.setCount : oldCount;
        }
    }

    @GwtIncompatible("java.io.ObjectOutputStream")
    private void writeObject(ObjectOutputStream stream) throws IOException {
        stream.defaultWriteObject();
        stream.writeObject(elementSet().comparator());
        Serialization.writeMultiset(this, stream);
    }

    @GwtIncompatible("java.io.ObjectInputStream")
    private void readObject(ObjectInputStream stream) throws IOException, ClassNotFoundException {
        stream.defaultReadObject();
        Comparator<? super E> comparator = (Comparator) stream.readObject();
        Serialization.getFieldSetter(AbstractSortedMultiset.class, "comparator").set(this, (Object) comparator);
        Serialization.getFieldSetter(TreeMultiset.class, "range").set(this, (Object) GeneralRange.all(comparator));
        Serialization.getFieldSetter(TreeMultiset.class, "rootReference").set(this, (Object) new Reference());
        Serialization.populateMultiset(this, stream);
    }
}
