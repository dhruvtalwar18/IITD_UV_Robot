package javax.jmdns.impl.constants;

public enum DNSState {
    PROBING_1("probing 1", StateClass.probing),
    PROBING_2("probing 2", StateClass.probing),
    PROBING_3("probing 3", StateClass.probing),
    ANNOUNCING_1("announcing 1", StateClass.announcing),
    ANNOUNCING_2("announcing 2", StateClass.announcing),
    ANNOUNCED("announced", StateClass.announced),
    CANCELING_1("canceling 1", StateClass.canceling),
    CANCELING_2("canceling 2", StateClass.canceling),
    CANCELING_3("canceling 3", StateClass.canceling),
    CANCELED("canceled", StateClass.canceled),
    CLOSING("closing", StateClass.closing),
    CLOSED("closed", StateClass.closed);
    
    private final String _name;
    private final StateClass _state;

    private enum StateClass {
        probing,
        announcing,
        announced,
        canceling,
        canceled,
        closing,
        closed
    }

    private DNSState(String name, StateClass state) {
        this._name = name;
        this._state = state;
    }

    public final String toString() {
        return this._name;
    }

    public final DNSState advance() {
        switch (this) {
            case PROBING_1:
                return PROBING_2;
            case PROBING_2:
                return PROBING_3;
            case PROBING_3:
                return ANNOUNCING_1;
            case ANNOUNCING_1:
                return ANNOUNCING_2;
            case ANNOUNCING_2:
                return ANNOUNCED;
            case ANNOUNCED:
                return ANNOUNCED;
            case CANCELING_1:
                return CANCELING_2;
            case CANCELING_2:
                return CANCELING_3;
            case CANCELING_3:
                return CANCELED;
            case CANCELED:
                return CANCELED;
            case CLOSING:
                return CLOSED;
            case CLOSED:
                return CLOSED;
            default:
                return this;
        }
    }

    public final DNSState revert() {
        switch (this) {
            case PROBING_1:
            case PROBING_2:
            case PROBING_3:
            case ANNOUNCING_1:
            case ANNOUNCING_2:
            case ANNOUNCED:
                return PROBING_1;
            case CANCELING_1:
            case CANCELING_2:
            case CANCELING_3:
                return CANCELING_1;
            case CANCELED:
                return CANCELED;
            case CLOSING:
                return CLOSING;
            case CLOSED:
                return CLOSED;
            default:
                return this;
        }
    }

    public final boolean isProbing() {
        return this._state == StateClass.probing;
    }

    public final boolean isAnnouncing() {
        return this._state == StateClass.announcing;
    }

    public final boolean isAnnounced() {
        return this._state == StateClass.announced;
    }

    public final boolean isCanceling() {
        return this._state == StateClass.canceling;
    }

    public final boolean isCanceled() {
        return this._state == StateClass.canceled;
    }

    public final boolean isClosing() {
        return this._state == StateClass.closing;
    }

    public final boolean isClosed() {
        return this._state == StateClass.closed;
    }
}
