package org.ros.internal.message.definition;

import com.google.common.base.Preconditions;
import com.google.common.collect.Lists;
import java.util.List;

public class MessageDefinitionTupleParser {
    private static final String SEPARATOR = "---";

    public static List<String> parse(String definition, int size) {
        Preconditions.checkNotNull(definition);
        List<String> definitions = Lists.newArrayList();
        StringBuilder current = new StringBuilder();
        StringBuilder current2 = current;
        for (String line : definition.split("\n")) {
            if (line.startsWith(SEPARATOR)) {
                definitions.add(current2.toString());
                current2 = new StringBuilder();
            } else {
                current2.append(line);
                current2.append("\n");
            }
        }
        if (current2.length() > 0) {
            current2.deleteCharAt(current2.length() - 1);
        }
        definitions.add(current2.toString());
        Preconditions.checkState(size == -1 || definitions.size() <= size, String.format("Message tuple exceeds expected size: %d > %d", new Object[]{Integer.valueOf(definitions.size()), Integer.valueOf(size)}));
        while (definitions.size() < size) {
            definitions.add("");
        }
        return definitions;
    }
}
