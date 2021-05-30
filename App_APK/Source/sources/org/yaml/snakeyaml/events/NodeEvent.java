package org.yaml.snakeyaml.events;

import org.yaml.snakeyaml.error.Mark;

public abstract class NodeEvent extends Event {
    private final String anchor;

    public NodeEvent(String anchor2, Mark startMark, Mark endMark) {
        super(startMark, endMark);
        this.anchor = anchor2;
    }

    public String getAnchor() {
        return this.anchor;
    }

    /* access modifiers changed from: protected */
    public String getArguments() {
        return "anchor=" + this.anchor;
    }
}
