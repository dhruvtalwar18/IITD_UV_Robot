package org.yaml.snakeyaml.nodes;

import java.util.List;
import org.yaml.snakeyaml.error.Mark;

public class SequenceNode extends CollectionNode {
    private final List<Node> value;

    public SequenceNode(Tag tag, boolean resolved, List<Node> value2, Mark startMark, Mark endMark, Boolean flowStyle) {
        super(tag, startMark, endMark, flowStyle);
        if (value2 != null) {
            this.value = value2;
            this.resolved = resolved;
            return;
        }
        throw new NullPointerException("value in a Node is required.");
    }

    public SequenceNode(Tag tag, List<Node> value2, Boolean flowStyle) {
        this(tag, true, value2, (Mark) null, (Mark) null, flowStyle);
    }

    public NodeId getNodeId() {
        return NodeId.sequence;
    }

    public List<Node> getValue() {
        return this.value;
    }

    public void setListType(Class<? extends Object> listType) {
        for (Node node : this.value) {
            node.setType(listType);
        }
    }

    public String toString() {
        return "<" + getClass().getName() + " (tag=" + getTag() + ", value=" + getValue() + ")>";
    }
}
