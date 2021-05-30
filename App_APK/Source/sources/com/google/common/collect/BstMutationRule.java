package com.google.common.collect;

import com.google.common.annotations.GwtCompatible;
import com.google.common.base.Preconditions;
import com.google.common.collect.BstNode;

@GwtCompatible
final class BstMutationRule<K, N extends BstNode<K, N>> {
    private final BstBalancePolicy<N> balancePolicy;
    private final BstModifier<K, N> modifier;
    private final BstNodeFactory<N> nodeFactory;

    public static <K, N extends BstNode<K, N>> BstMutationRule<K, N> createRule(BstModifier<K, N> modifier2, BstBalancePolicy<N> balancePolicy2, BstNodeFactory<N> nodeFactory2) {
        return new BstMutationRule<>(modifier2, balancePolicy2, nodeFactory2);
    }

    private BstMutationRule(BstModifier<K, N> modifier2, BstBalancePolicy<N> balancePolicy2, BstNodeFactory<N> nodeFactory2) {
        this.balancePolicy = (BstBalancePolicy) Preconditions.checkNotNull(balancePolicy2);
        this.nodeFactory = (BstNodeFactory) Preconditions.checkNotNull(nodeFactory2);
        this.modifier = (BstModifier) Preconditions.checkNotNull(modifier2);
    }

    public BstModifier<K, N> getModifier() {
        return this.modifier;
    }

    public BstBalancePolicy<N> getBalancePolicy() {
        return this.balancePolicy;
    }

    public BstNodeFactory<N> getNodeFactory() {
        return this.nodeFactory;
    }
}
