package org.yaml.snakeyaml.reader;

import org.yaml.snakeyaml.error.YAMLException;

public class ReaderException extends YAMLException {
    private static final long serialVersionUID = 8710781187529689083L;
    private final char character;
    private final String name;
    private final int position;

    public ReaderException(String name2, int position2, char character2, String message) {
        super(message);
        this.name = name2;
        this.character = character2;
        this.position = position2;
    }

    public String getName() {
        return this.name;
    }

    public char getCharacter() {
        return this.character;
    }

    public int getPosition() {
        return this.position;
    }

    public String toString() {
        return "unacceptable character '" + this.character + "' (0x" + Integer.toHexString(this.character).toUpperCase() + ") " + getMessage() + "\nin \"" + this.name + "\", position " + this.position;
    }
}
