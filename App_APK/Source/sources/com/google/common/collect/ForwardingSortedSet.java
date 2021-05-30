package com.google.common.collect;

import com.google.common.annotations.Beta;
import com.google.common.annotations.GwtCompatible;
import java.util.Comparator;
import java.util.Iterator;
import java.util.NoSuchElementException;
import java.util.SortedSet;
import javax.annotation.Nullable;

@GwtCompatible
public abstract class ForwardingSortedSet<E> extends ForwardingSet<E> implements SortedSet<E> {
    /* access modifiers changed from: protected */
    public abstract SortedSet<E> delegate();

    protected ForwardingSortedSet() {
    }

    public Comparator<? super E> comparator() {
        return delegate().comparator();
    }

    public E first() {
        return delegate().first();
    }

    public SortedSet<E> headSet(E toElement) {
        return delegate().headSet(toElement);
    }

    public E last() {
        return delegate().last();
    }

    public SortedSet<E> subSet(E fromElement, E toElement) {
        return delegate().subSet(fromElement, toElement);
    }

    public SortedSet<E> tailSet(E fromElement) {
        return delegate().tailSet(fromElement);
    }

    private int unsafeCompare(Object o1, Object o2) {
        Comparator<? super E> comparator = comparator();
        return comparator == null ? ((Comparable) o1).compareTo(o2) : comparator.compare(o1, o2);
    }

    /* access modifiers changed from: protected */
    @Beta
    public boolean standardContains(@Nullable Object object) {
        try {
            if (unsafeCompare(tailSet(object).first(), object) == 0) {
                return true;
            }
            return false;
        } catch (ClassCastException e) {
            return false;
        } catch (NoSuchElementException e2) {
            return false;
        } catch (NullPointerException e3) {
            return false;
        }
    }

    /* access modifiers changed from: protected */
    @Beta
    public boolean standardRemove(@Nullable Object object) {
        try {
            Iterator<Object> iterator = tailSet(object).iterator();
            if (!iterator.hasNext() || unsafeCompare(iterator.next(), object) != 0) {
                return false;
            }
            iterator.remove();
            return true;
        } catch (ClassCastException e) {
            return false;
        } catch (NullPointerException e2) {
            return false;
        }
    }

    /* access modifiers changed from: protected */
    @Beta
    public SortedSet<E> standardSubSet(E fromElement, E toElement) {
        return tailSet(fromElement).headSet(toElement);
    }
}
