package org.yaml.snakeyaml.nodes;

import org.yaml.snakeyaml.error.Mark;

public class ScalarNode extends Node {
    private Character style;
    private String value;

    public ScalarNode(Tag tag, String value2, Mark startMark, Mark endMark, Character style2) {
        this(tag, true, value2, startMark, endMark, style2);
    }

    public ScalarNode(Tag tag, boolean resolved, String value2, Mark startMark, Mark endMark, Character style2) {
        super(tag, startMark, endMark);
        if (value2 != null) {
            this.value = value2;
            this.style = style2;
            this.resolved = resolved;
            return;
        }
        throw new NullPointerException("value in a Node is required.");
    }

    public Character getStyle() {
        return this.style;
    }

    public NodeId getNodeId() {
        return NodeId.scalar;
    }

    public String getValue() {
        return this.value;
    }

    public String toString() {
        return "<" + getClass().getName() + " (tag=" + getTag() + ", value=" + getValue() + ")>";
    }
}
