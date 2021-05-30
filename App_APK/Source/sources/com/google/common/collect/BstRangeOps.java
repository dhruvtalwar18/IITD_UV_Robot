package com.google.common.collect;

import com.google.common.annotations.GwtCompatible;
import com.google.common.base.Preconditions;
import javax.annotation.Nullable;

@GwtCompatible
final class BstRangeOps {
    public static <K, N extends BstNode<K, N>> long totalInRange(BstAggregate<? super N> aggregate, GeneralRange<K> range, @Nullable N root) {
        Preconditions.checkNotNull(aggregate);
        Preconditions.checkNotNull(range);
        if (root == null || range.isEmpty()) {
            return 0;
        }
        long total = aggregate.treeValue(root);
        if (range.hasLowerBound()) {
            total -= totalBeyondRangeToSide(aggregate, range, BstSide.LEFT, root);
        }
        if (range.hasUpperBound()) {
            return total - totalBeyondRangeToSide(aggregate, range, BstSide.RIGHT, root);
        }
        return total;
    }

    private static <K, N extends BstNode<K, N>> long totalBeyondRangeToSide(BstAggregate<? super N> aggregate, GeneralRange<K> range, BstSide side, @Nullable N root) {
        long accum = 0;
        while (root != null) {
            if (beyond(range, root.getKey(), side)) {
                accum = accum + ((long) aggregate.entryValue(root)) + aggregate.treeValue(root.childOrNull(side));
                root = root.childOrNull(side.other());
            } else {
                root = root.childOrNull(side);
            }
        }
        return accum;
    }

    @Nullable
    public static <K, N extends BstNode<K, N>> N minusRange(GeneralRange<K> range, BstBalancePolicy<N> balancePolicy, BstNodeFactory<N> nodeFactory, @Nullable N root) {
        Preconditions.checkNotNull(range);
        Preconditions.checkNotNull(balancePolicy);
        Preconditions.checkNotNull(nodeFactory);
        N lower = null;
        N higher = range.hasUpperBound() ? subTreeBeyondRangeToSide(range, balancePolicy, nodeFactory, BstSide.RIGHT, root) : null;
        if (range.hasLowerBound()) {
            lower = subTreeBeyondRangeToSide(range, balancePolicy, nodeFactory, BstSide.LEFT, root);
        }
        return balancePolicy.combine(nodeFactory, lower, higher);
    }

    @Nullable
    private static <K, N extends BstNode<K, N>> N subTreeBeyondRangeToSide(GeneralRange<K> range, BstBalancePolicy<N> balancePolicy, BstNodeFactory<N> nodeFactory, BstSide side, @Nullable N root) {
        if (root == null) {
            return null;
        }
        if (!beyond(range, root.getKey(), side)) {
            return subTreeBeyondRangeToSide(range, balancePolicy, nodeFactory, side, root.childOrNull(side));
        }
        N left = root.childOrNull(BstSide.LEFT);
        N right = root.childOrNull(BstSide.RIGHT);
        switch (side) {
            case LEFT:
                right = subTreeBeyondRangeToSide(range, balancePolicy, nodeFactory, BstSide.LEFT, right);
                break;
            case RIGHT:
                left = subTreeBeyondRangeToSide(range, balancePolicy, nodeFactory, BstSide.RIGHT, left);
                break;
            default:
                throw new AssertionError();
        }
        return balancePolicy.balance(nodeFactory, root, left, right);
    }

    @Nullable
    public static <K, N extends BstNode<K, N>, P extends BstPath<N, P>> P furthestPath(GeneralRange<K> range, BstSide side, BstPathFactory<N, P> pathFactory, @Nullable N root) {
        Preconditions.checkNotNull(range);
        Preconditions.checkNotNull(pathFactory);
        Preconditions.checkNotNull(side);
        if (root == null) {
            return null;
        }
        return furthestPath(range, side, pathFactory, pathFactory.initialPath(root));
    }

    private static <K, N extends BstNode<K, N>, P extends BstPath<N, P>> P furthestPath(GeneralRange<K> range, BstSide side, BstPathFactory<N, P> pathFactory, P currentPath) {
        P alphaPath;
        N tip = currentPath.getTip();
        K tipKey = tip.getKey();
        if (beyond(range, tipKey, side)) {
            if (tip.hasChild(side.other())) {
                return furthestPath(range, side, pathFactory, pathFactory.extension(currentPath, side.other()));
            }
            return null;
        } else if (tip.hasChild(side) && (alphaPath = furthestPath(range, side, pathFactory, pathFactory.extension(currentPath, side))) != null) {
            return alphaPath;
        } else {
            if (beyond(range, tipKey, side.other())) {
                return null;
            }
            return currentPath;
        }
    }

    public static <K> boolean beyond(GeneralRange<K> range, @Nullable K key, BstSide side) {
        Preconditions.checkNotNull(range);
        switch (side) {
            case LEFT:
                return range.tooLow(key);
            case RIGHT:
                return range.tooHigh(key);
            default:
                throw new AssertionError();
        }
    }

    private BstRangeOps() {
    }
}
