package com.google.common.collect;

import com.google.common.annotations.GwtCompatible;
import com.google.common.base.Optional;
import com.google.common.base.Preconditions;
import com.google.common.collect.BstNode;
import java.util.NoSuchElementException;
import javax.annotation.Nullable;

@GwtCompatible
final class BstInOrderPath<N extends BstNode<?, N>> extends BstPath<N, BstInOrderPath<N>> {
    static final /* synthetic */ boolean $assertionsDisabled = false;
    private transient Optional<BstInOrderPath<N>> nextInOrder;
    private transient Optional<BstInOrderPath<N>> prevInOrder;
    private final BstSide sideExtension;

    public static <N extends BstNode<?, N>> BstPathFactory<N, BstInOrderPath<N>> inOrderFactory() {
        return new BstPathFactory<N, BstInOrderPath<N>>() {
            public BstInOrderPath<N> extension(BstInOrderPath<N> path, BstSide side) {
                return BstInOrderPath.extension(path, side);
            }

            public BstInOrderPath<N> initialPath(N root) {
                return new BstInOrderPath<>(root, (BstSide) null, (BstInOrderPath) null);
            }
        };
    }

    /* access modifiers changed from: private */
    public static <N extends BstNode<?, N>> BstInOrderPath<N> extension(BstInOrderPath<N> path, BstSide side) {
        Preconditions.checkNotNull(path);
        return new BstInOrderPath<>(path.getTip().getChild(side), side, path);
    }

    private BstInOrderPath(N tip, @Nullable BstSide sideExtension2, @Nullable BstInOrderPath<N> tail) {
        super(tip, tail);
        this.sideExtension = sideExtension2;
    }

    /* JADX WARNING: type inference failed for: r1v3, types: [com.google.common.collect.BstPath] */
    /* JADX WARNING: Multi-variable type inference failed */
    /* Code decompiled incorrectly, please refer to instructions dump. */
    private com.google.common.base.Optional<com.google.common.collect.BstInOrderPath<N>> computeNextInOrder(com.google.common.collect.BstSide r4) {
        /*
            r3 = this;
            com.google.common.collect.BstNode r0 = r3.getTip()
            boolean r0 = r0.hasChild(r4)
            if (r0 == 0) goto L_0x0026
            com.google.common.collect.BstInOrderPath r0 = extension(r3, r4)
            com.google.common.collect.BstSide r1 = r4.other()
        L_0x0012:
            com.google.common.collect.BstNode r2 = r0.getTip()
            boolean r2 = r2.hasChild(r1)
            if (r2 == 0) goto L_0x0021
            com.google.common.collect.BstInOrderPath r0 = extension(r0, r1)
            goto L_0x0012
        L_0x0021:
            com.google.common.base.Optional r2 = com.google.common.base.Optional.of(r0)
            return r2
        L_0x0026:
            r0 = r3
        L_0x0027:
            com.google.common.collect.BstSide r1 = r0.sideExtension
            if (r1 != r4) goto L_0x0033
            com.google.common.collect.BstPath r1 = r0.getPrefix()
            r0 = r1
            com.google.common.collect.BstInOrderPath r0 = (com.google.common.collect.BstInOrderPath) r0
            goto L_0x0027
        L_0x0033:
            com.google.common.collect.BstPath r1 = r0.prefixOrNull()
            r0 = r1
            com.google.common.collect.BstInOrderPath r0 = (com.google.common.collect.BstInOrderPath) r0
            com.google.common.base.Optional r1 = com.google.common.base.Optional.fromNullable(r0)
            return r1
        */
        throw new UnsupportedOperationException("Method not decompiled: com.google.common.collect.BstInOrderPath.computeNextInOrder(com.google.common.collect.BstSide):com.google.common.base.Optional");
    }

    private Optional<BstInOrderPath<N>> nextInOrder(BstSide side) {
        switch (side) {
            case LEFT:
                Optional<BstInOrderPath<N>> result = this.prevInOrder;
                if (result != null) {
                    return result;
                }
                Optional<BstInOrderPath<N>> computeNextInOrder = computeNextInOrder(side);
                this.prevInOrder = computeNextInOrder;
                return computeNextInOrder;
            case RIGHT:
                Optional<BstInOrderPath<N>> result2 = this.nextInOrder;
                if (result2 != null) {
                    return result2;
                }
                Optional<BstInOrderPath<N>> computeNextInOrder2 = computeNextInOrder(side);
                this.nextInOrder = computeNextInOrder2;
                return computeNextInOrder2;
            default:
                throw new AssertionError();
        }
    }

    public boolean hasNext(BstSide side) {
        return nextInOrder(side).isPresent();
    }

    public BstInOrderPath<N> next(BstSide side) {
        if (hasNext(side)) {
            return (BstInOrderPath) nextInOrder(side).get();
        }
        throw new NoSuchElementException();
    }

    public BstSide getSideOfExtension() {
        return this.sideExtension;
    }
}
