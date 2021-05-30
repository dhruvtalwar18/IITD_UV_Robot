package com.google.common.collect;

import com.google.common.annotations.GwtCompatible;
import com.google.common.collect.Multiset;
import com.google.common.collect.Multisets;
import java.util.Comparator;
import java.util.Iterator;
import java.util.NoSuchElementException;
import java.util.Set;
import java.util.SortedSet;

@GwtCompatible
final class SortedMultisets {
    private SortedMultisets() {
    }

    static abstract class ElementSet<E> extends Multisets.ElementSet<E> implements SortedSet<E> {
        /* access modifiers changed from: package-private */
        public abstract SortedMultiset<E> multiset();

        ElementSet() {
        }

        public Comparator<? super E> comparator() {
            return multiset().comparator();
        }

        public SortedSet<E> subSet(E fromElement, E toElement) {
            return multiset().subMultiset(fromElement, BoundType.CLOSED, toElement, BoundType.OPEN).elementSet();
        }

        public SortedSet<E> headSet(E toElement) {
            return multiset().headMultiset(toElement, BoundType.OPEN).elementSet();
        }

        public SortedSet<E> tailSet(E fromElement) {
            return multiset().tailMultiset(fromElement, BoundType.CLOSED).elementSet();
        }

        public E first() {
            return SortedMultisets.getElementOrThrow(multiset().firstEntry());
        }

        public E last() {
            return SortedMultisets.getElementOrThrow(multiset().lastEntry());
        }
    }

    /* access modifiers changed from: private */
    public static <E> E getElementOrThrow(Multiset.Entry<E> entry) {
        if (entry != null) {
            return entry.getElement();
        }
        throw new NoSuchElementException();
    }

    static abstract class DescendingMultiset<E> extends ForwardingMultiset<E> implements SortedMultiset<E> {
        private transient Comparator<? super E> comparator;
        private transient SortedSet<E> elementSet;
        private transient Set<Multiset.Entry<E>> entrySet;

        /* access modifiers changed from: package-private */
        public abstract Iterator<Multiset.Entry<E>> entryIterator();

        /* access modifiers changed from: package-private */
        public abstract SortedMultiset<E> forwardMultiset();

        DescendingMultiset() {
        }

        public Comparator<? super E> comparator() {
            Comparator<? super E> result = this.comparator;
            if (result != null) {
                return result;
            }
            Ordering reverse = Ordering.from(forwardMultiset().comparator()).reverse();
            this.comparator = reverse;
            return reverse;
        }

        public SortedSet<E> elementSet() {
            SortedSet<E> result = this.elementSet;
            if (result != null) {
                return result;
            }
            AnonymousClass1 r1 = new ElementSet<E>() {
                /* access modifiers changed from: package-private */
                public SortedMultiset<E> multiset() {
                    return DescendingMultiset.this;
                }
            };
            this.elementSet = r1;
            return r1;
        }

        public Multiset.Entry<E> pollFirstEntry() {
            return forwardMultiset().pollLastEntry();
        }

        public Multiset.Entry<E> pollLastEntry() {
            return forwardMultiset().pollFirstEntry();
        }

        public SortedMultiset<E> headMultiset(E toElement, BoundType boundType) {
            return forwardMultiset().tailMultiset(toElement, boundType).descendingMultiset();
        }

        public SortedMultiset<E> subMultiset(E fromElement, BoundType fromBoundType, E toElement, BoundType toBoundType) {
            return forwardMultiset().subMultiset(toElement, toBoundType, fromElement, fromBoundType).descendingMultiset();
        }

        public SortedMultiset<E> tailMultiset(E fromElement, BoundType boundType) {
            return forwardMultiset().headMultiset(fromElement, boundType).descendingMultiset();
        }

        /* access modifiers changed from: protected */
        public Multiset<E> delegate() {
            return forwardMultiset();
        }

        public SortedMultiset<E> descendingMultiset() {
            return forwardMultiset();
        }

        public Multiset.Entry<E> firstEntry() {
            return forwardMultiset().lastEntry();
        }

        public Multiset.Entry<E> lastEntry() {
            return forwardMultiset().firstEntry();
        }

        public Set<Multiset.Entry<E>> entrySet() {
            Set<Multiset.Entry<E>> result = this.entrySet;
            if (result != null) {
                return result;
            }
            Set<Multiset.Entry<E>> createEntrySet = createEntrySet();
            this.entrySet = createEntrySet;
            return createEntrySet;
        }

        /* access modifiers changed from: package-private */
        public Set<Multiset.Entry<E>> createEntrySet() {
            return new Multisets.EntrySet<E>() {
                /* access modifiers changed from: package-private */
                public Multiset<E> multiset() {
                    return DescendingMultiset.this;
                }

                public Iterator<Multiset.Entry<E>> iterator() {
                    return DescendingMultiset.this.entryIterator();
                }

                public int size() {
                    return DescendingMultiset.this.forwardMultiset().entrySet().size();
                }
            };
        }

        public Iterator<E> iterator() {
            return Multisets.iteratorImpl(this);
        }

        public Object[] toArray() {
            return standardToArray();
        }

        public <T> T[] toArray(T[] array) {
            return standardToArray(array);
        }

        public String toString() {
            return entrySet().toString();
        }
    }
}
