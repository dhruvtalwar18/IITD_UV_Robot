package org.ros.internal.node.response;

import com.google.common.collect.Lists;
import java.util.List;
import org.ros.exception.RemoteException;
import org.ros.exception.RosRuntimeException;

public class Response<T> {
    private final T result;
    private final StatusCode statusCode;
    private final String statusMessage;

    public static <T> Response<T> newError(String message, T value) {
        return new Response<>(StatusCode.ERROR, message, value);
    }

    public static <T> Response<T> newFailure(String message, T value) {
        return new Response<>(StatusCode.FAILURE, message, value);
    }

    public static <T> Response<T> newSuccess(String message, T value) {
        return new Response<>(StatusCode.SUCCESS, message, value);
    }

    public static <T> Response<T> fromListCheckedFailure(List<Object> response, ResultFactory<T> resultFactory) throws RemoteException {
        try {
            StatusCode statusCode2 = StatusCode.fromInt(((Integer) response.get(0)).intValue());
            String message = (String) response.get(1);
            if (statusCode2 != StatusCode.FAILURE) {
                try {
                    return new Response<>(statusCode2, message, resultFactory.newFromValue(response.get(2)));
                } catch (ClassCastException e) {
                    throw new RosRuntimeException("Remote side did not return correct value type.", e);
                }
            } else {
                throw new RemoteException(statusCode2, message);
            }
        } catch (ClassCastException e2) {
            throw new RosRuntimeException("Remote side did not return correct type (status code/message).", e2);
        }
    }

    public static <T> Response<T> fromListChecked(List<Object> response, ResultFactory<T> resultFactory) throws RemoteException {
        try {
            StatusCode statusCode2 = StatusCode.fromInt(((Integer) response.get(0)).intValue());
            String message = (String) response.get(1);
            if (statusCode2 == StatusCode.SUCCESS) {
                try {
                    return new Response<>(statusCode2, message, resultFactory.newFromValue(response.get(2)));
                } catch (ClassCastException e) {
                    throw new RosRuntimeException("Remote side did not return correct value type.", e);
                }
            } else {
                throw new RemoteException(statusCode2, message);
            }
        } catch (ClassCastException e2) {
            throw new RosRuntimeException("Remote side did not return correct type (status code/message).", e2);
        }
    }

    public Response(int statusCode2, String statusMessage2, T value) {
        this(StatusCode.fromInt(statusCode2), statusMessage2, value);
    }

    public Response(StatusCode statusCode2, String statusMessage2, T value) {
        this.statusCode = statusCode2;
        this.statusMessage = statusMessage2;
        this.result = value;
    }

    public List<Object> toList() {
        Object[] objArr = new Object[3];
        objArr[0] = Integer.valueOf(this.statusCode.toInt());
        objArr[1] = this.statusMessage;
        objArr[2] = this.result == null ? "null" : this.result;
        return Lists.newArrayList((E[]) objArr);
    }

    public StatusCode getStatusCode() {
        return this.statusCode;
    }

    public String getStatusMessage() {
        return this.statusMessage;
    }

    public T getResult() {
        return this.result;
    }

    public String toString() {
        return "Response<" + this.statusCode + ", " + this.statusMessage + ", " + this.result + ">";
    }

    public boolean isSuccess() {
        return this.statusCode == StatusCode.SUCCESS;
    }
}
