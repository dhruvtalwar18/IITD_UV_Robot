package org.yaml.snakeyaml.nodes;

public final class NodeTuple {
    private Node keyNode;
    private Node valueNode;

    public NodeTuple(Node keyNode2, Node valueNode2) {
        if (keyNode2 == null || valueNode2 == null) {
            throw new NullPointerException("Nodes must be provided.");
        }
        this.keyNode = keyNode2;
        this.valueNode = valueNode2;
    }

    public final Node getKeyNode() {
        return this.keyNode;
    }

    public final Node getValueNode() {
        return this.valueNode;
    }

    public String toString() {
        return "<NodeTuple keyNode=" + this.keyNode.toString() + "; valueNode=" + this.valueNode.toString() + ">";
    }
}
