package org.yaml.snakeyaml.nodes;

public class AnchorNode extends Node {
    private Node realNode;

    public AnchorNode(Node realNode2) {
        super(realNode2.getTag(), realNode2.getStartMark(), realNode2.getEndMark());
        this.realNode = realNode2;
    }

    public NodeId getNodeId() {
        return NodeId.anchor;
    }

    public Node getRealNode() {
        return this.realNode;
    }
}
