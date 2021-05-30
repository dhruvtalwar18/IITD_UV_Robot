package org.yaml.snakeyaml.nodes;

import java.util.List;
import org.yaml.snakeyaml.error.Mark;

public class MappingNode extends CollectionNode {
    private boolean merged;
    private List<NodeTuple> value;

    public MappingNode(Tag tag, boolean resolved, List<NodeTuple> value2, Mark startMark, Mark endMark, Boolean flowStyle) {
        super(tag, startMark, endMark, flowStyle);
        this.merged = false;
        if (value2 != null) {
            this.value = value2;
            this.resolved = resolved;
            return;
        }
        throw new NullPointerException("value in a Node is required.");
    }

    public MappingNode(Tag tag, List<NodeTuple> value2, Boolean flowStyle) {
        this(tag, true, value2, (Mark) null, (Mark) null, flowStyle);
    }

    public NodeId getNodeId() {
        return NodeId.mapping;
    }

    public List<NodeTuple> getValue() {
        return this.value;
    }

    public void setValue(List<NodeTuple> merge) {
        this.value = merge;
    }

    public void setOnlyKeyType(Class<? extends Object> keyType) {
        for (NodeTuple nodes : this.value) {
            nodes.getKeyNode().setType(keyType);
        }
    }

    public void setTypes(Class<? extends Object> keyType, Class<? extends Object> valueType) {
        for (NodeTuple nodes : this.value) {
            nodes.getValueNode().setType(valueType);
            nodes.getKeyNode().setType(keyType);
        }
    }

    public String toString() {
        StringBuilder buf = new StringBuilder();
        for (NodeTuple node : getValue()) {
            buf.append("{ key=");
            buf.append(node.getKeyNode());
            buf.append("; value=");
            if (node.getValueNode() instanceof CollectionNode) {
                buf.append(System.identityHashCode(node.getValueNode()));
            } else {
                buf.append(node.toString());
            }
            buf.append(" }");
        }
        String values = buf.toString();
        return "<" + getClass().getName() + " (tag=" + getTag() + ", values=" + values + ")>";
    }

    public void setMerged(boolean merged2) {
        this.merged = merged2;
    }

    public boolean isMerged() {
        return this.merged;
    }
}
