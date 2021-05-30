package org.yaml.snakeyaml.events;

import org.yaml.snakeyaml.error.Mark;
import org.yaml.snakeyaml.events.Event;

public final class DocumentEndEvent extends Event {
    private final boolean explicit;

    public DocumentEndEvent(Mark startMark, Mark endMark, boolean explicit2) {
        super(startMark, endMark);
        this.explicit = explicit2;
    }

    public boolean getExplicit() {
        return this.explicit;
    }

    public boolean is(Event.ID id) {
        return Event.ID.DocumentEnd == id;
    }
}
