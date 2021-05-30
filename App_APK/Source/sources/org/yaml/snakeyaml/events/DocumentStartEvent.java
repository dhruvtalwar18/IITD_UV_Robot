package org.yaml.snakeyaml.events;

import java.util.Map;
import org.yaml.snakeyaml.error.Mark;
import org.yaml.snakeyaml.events.Event;

public final class DocumentStartEvent extends Event {
    private final boolean explicit;
    private final Map<String, String> tags;
    private final Integer[] version;

    public DocumentStartEvent(Mark startMark, Mark endMark, boolean explicit2, Integer[] version2, Map<String, String> tags2) {
        super(startMark, endMark);
        this.explicit = explicit2;
        this.version = version2;
        this.tags = tags2;
    }

    public boolean getExplicit() {
        return this.explicit;
    }

    public Integer[] getVersion() {
        return this.version;
    }

    public Map<String, String> getTags() {
        return this.tags;
    }

    public boolean is(Event.ID id) {
        return Event.ID.DocumentStart == id;
    }
}
