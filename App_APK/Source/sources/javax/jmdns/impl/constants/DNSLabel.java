package javax.jmdns.impl.constants;

public enum DNSLabel {
    Unknown("", 128),
    Standard("standard label", 0),
    Compressed("compressed label", LABEL_MASK),
    Extended("extended label", 64);
    
    static final int LABEL_MASK = 192;
    static final int LABEL_NOT_MASK = 63;
    private final String _externalName;
    private final int _index;

    private DNSLabel(String name, int index) {
        this._externalName = name;
        this._index = index;
    }

    public String externalName() {
        return this._externalName;
    }

    public int indexValue() {
        return this._index;
    }

    public static DNSLabel labelForByte(int index) {
        int maskedIndex = index & LABEL_MASK;
        for (DNSLabel aLabel : values()) {
            if (aLabel._index == maskedIndex) {
                return aLabel;
            }
        }
        return Unknown;
    }

    public static int labelValue(int index) {
        return index & 63;
    }

    public String toString() {
        return name() + " index " + indexValue();
    }
}
