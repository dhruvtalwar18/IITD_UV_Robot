package com.google.common.collect;

import com.google.common.annotations.GwtCompatible;
import com.google.common.base.Preconditions;
import javax.annotation.Nullable;

@GwtCompatible
final class BstCountBasedBalancePolicies {
    private static final int SECOND_ROTATE_RATIO = 2;
    private static final int SINGLE_ROTATE_RATIO = 4;

    private BstCountBasedBalancePolicies() {
    }

    public static <N extends BstNode<?, N>> BstBalancePolicy<N> noRebalancePolicy(final BstAggregate<N> countAggregate) {
        Preconditions.checkNotNull(countAggregate);
        return new BstBalancePolicy<N>() {
            public N balance(BstNodeFactory<N> nodeFactory, N source, @Nullable N left, @Nullable N right) {
                return ((BstNodeFactory) Preconditions.checkNotNull(nodeFactory)).createNode(source, left, right);
            }

            @Nullable
            public N combine(BstNodeFactory<N> nodeFactory, @Nullable N left, @Nullable N right) {
                if (left == null) {
                    return right;
                }
                if (right == null) {
                    return left;
                }
                if (countAggregate.treeValue(left) > countAggregate.treeValue(right)) {
                    return nodeFactory.createNode(left, left.childOrNull(BstSide.LEFT), combine(nodeFactory, left.childOrNull(BstSide.RIGHT), right));
                }
                return nodeFactory.createNode(right, combine(nodeFactory, left, right.childOrNull(BstSide.LEFT)), right.childOrNull(BstSide.RIGHT));
            }
        };
    }

    public static <K, N extends BstNode<K, N>> BstBalancePolicy<N> singleRebalancePolicy(final BstAggregate<N> countAggregate) {
        Preconditions.checkNotNull(countAggregate);
        return new BstBalancePolicy<N>() {
            public N balance(BstNodeFactory<N> nodeFactory, N source, @Nullable N left, @Nullable N right) {
                long countL = countAggregate.treeValue(left);
                long countR = countAggregate.treeValue(right);
                if (countL + countR > 1) {
                    if (countR >= countL * 4) {
                        return rotateL(nodeFactory, source, left, right);
                    }
                    if (countL >= 4 * countR) {
                        return rotateR(nodeFactory, source, left, right);
                    }
                }
                return nodeFactory.createNode(source, left, right);
            }

            private N rotateL(BstNodeFactory<N> nodeFactory, N source, @Nullable N left, N right) {
                Preconditions.checkNotNull(right);
                N rl = right.childOrNull(BstSide.LEFT);
                N rr = right.childOrNull(BstSide.RIGHT);
                if (countAggregate.treeValue(rl) >= countAggregate.treeValue(rr) * 2) {
                    right = singleR(nodeFactory, right, rl, rr);
                }
                return singleL(nodeFactory, source, left, right);
            }

            private N rotateR(BstNodeFactory<N> nodeFactory, N source, N left, @Nullable N right) {
                Preconditions.checkNotNull(left);
                N lr = left.childOrNull(BstSide.RIGHT);
                N ll = left.childOrNull(BstSide.LEFT);
                if (countAggregate.treeValue(lr) >= countAggregate.treeValue(ll) * 2) {
                    left = singleL(nodeFactory, left, ll, lr);
                }
                return singleR(nodeFactory, source, left, right);
            }

            private N singleL(BstNodeFactory<N> nodeFactory, N source, @Nullable N left, N right) {
                Preconditions.checkNotNull(right);
                return nodeFactory.createNode(right, nodeFactory.createNode(source, left, right.childOrNull(BstSide.LEFT)), right.childOrNull(BstSide.RIGHT));
            }

            private N singleR(BstNodeFactory<N> nodeFactory, N source, N left, @Nullable N right) {
                Preconditions.checkNotNull(left);
                return nodeFactory.createNode(left, left.childOrNull(BstSide.LEFT), nodeFactory.createNode(source, left.childOrNull(BstSide.RIGHT), right));
            }

            @Nullable
            public N combine(BstNodeFactory<N> nodeFactory, @Nullable N left, @Nullable N right) {
                N newRootSource;
                if (left == null) {
                    return right;
                }
                if (right == null) {
                    return left;
                }
                if (countAggregate.treeValue(left) > countAggregate.treeValue(right)) {
                    BstMutationResult<K, N> extractLeftMax = BstOperations.extractMax(left, nodeFactory, this);
                    newRootSource = extractLeftMax.getOriginalTarget();
                    left = extractLeftMax.getChangedRoot();
                } else {
                    BstMutationResult<K, N> extractRightMin = BstOperations.extractMin(right, nodeFactory, this);
                    newRootSource = extractRightMin.getOriginalTarget();
                    right = extractRightMin.getChangedRoot();
                }
                return nodeFactory.createNode(newRootSource, left, right);
            }
        };
    }

    public static <K, N extends BstNode<K, N>> BstBalancePolicy<N> fullRebalancePolicy(final BstAggregate<N> countAggregate) {
        Preconditions.checkNotNull(countAggregate);
        final BstBalancePolicy<N> singleBalancePolicy = singleRebalancePolicy(countAggregate);
        return new BstBalancePolicy<N>() {
            public N balance(BstNodeFactory<N> nodeFactory, N source, @Nullable N left, @Nullable N right) {
                if (left == null) {
                    return BstOperations.insertMin(right, source, nodeFactory, singleBalancePolicy);
                }
                if (right == null) {
                    return BstOperations.insertMax(left, source, nodeFactory, singleBalancePolicy);
                }
                long countL = countAggregate.treeValue(left);
                long countR = countAggregate.treeValue(right);
                if (countL * 4 <= countR) {
                    return singleBalancePolicy.balance(nodeFactory, right, balance(nodeFactory, source, left, right.childOrNull(BstSide.LEFT)), right.childOrNull(BstSide.RIGHT));
                } else if (4 * countR > countL) {
                    return nodeFactory.createNode(source, left, right);
                } else {
                    return singleBalancePolicy.balance(nodeFactory, left, left.childOrNull(BstSide.LEFT), balance(nodeFactory, source, left.childOrNull(BstSide.RIGHT), right));
                }
            }

            @Nullable
            public N combine(BstNodeFactory<N> nodeFactory, @Nullable N left, @Nullable N right) {
                if (left == null) {
                    return right;
                }
                if (right == null) {
                    return left;
                }
                long countL = countAggregate.treeValue(left);
                long countR = countAggregate.treeValue(right);
                if (countL * 4 <= countR) {
                    return singleBalancePolicy.balance(nodeFactory, right, combine(nodeFactory, left, right.childOrNull(BstSide.LEFT)), right.childOrNull(BstSide.RIGHT));
                } else if (4 * countR > countL) {
                    return singleBalancePolicy.combine(nodeFactory, left, right);
                } else {
                    return singleBalancePolicy.balance(nodeFactory, left, left.childOrNull(BstSide.LEFT), combine(nodeFactory, left.childOrNull(BstSide.RIGHT), right));
                }
            }
        };
    }
}
