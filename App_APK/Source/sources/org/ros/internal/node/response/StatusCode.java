package org.ros.internal.node.response;

public enum StatusCode {
    ERROR(-1),
    FAILURE(0),
    SUCCESS(1);
    
    private final int intValue;

    private StatusCode(int value) {
        this.intValue = value;
    }

    public int toInt() {
        return this.intValue;
    }

    public static StatusCode fromInt(int intValue2) {
        if (intValue2 == -1) {
            return ERROR;
        }
        if (intValue2 != 1) {
            return FAILURE;
        }
        return SUCCESS;
    }

    public String toString() {
        switch (this) {
            case ERROR:
                return "Error";
            case SUCCESS:
                return "Success";
            default:
                return "Failure";
        }
    }
}
