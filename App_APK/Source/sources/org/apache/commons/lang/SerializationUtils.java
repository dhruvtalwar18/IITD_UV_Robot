package org.apache.commons.lang;

import java.io.ByteArrayInputStream;
import java.io.ByteArrayOutputStream;
import java.io.IOException;
import java.io.InputStream;
import java.io.ObjectInputStream;
import java.io.ObjectOutputStream;
import java.io.OutputStream;
import java.io.Serializable;

public class SerializationUtils {
    public static Object clone(Serializable object) {
        return deserialize(serialize(object));
    }

    public static void serialize(Serializable obj, OutputStream outputStream) {
        if (outputStream != null) {
            ObjectOutputStream out = null;
            try {
                out = new ObjectOutputStream(outputStream);
                out.writeObject(obj);
                try {
                    out.close();
                } catch (IOException e) {
                }
            } catch (IOException ex) {
                throw new SerializationException((Throwable) ex);
            } catch (Throwable th) {
                if (out != null) {
                    try {
                        out.close();
                    } catch (IOException e2) {
                    }
                }
                throw th;
            }
        } else {
            throw new IllegalArgumentException("The OutputStream must not be null");
        }
    }

    public static byte[] serialize(Serializable obj) {
        ByteArrayOutputStream baos = new ByteArrayOutputStream(512);
        serialize(obj, baos);
        return baos.toByteArray();
    }

    public static Object deserialize(InputStream inputStream) {
        if (inputStream != null) {
            ObjectInputStream in = null;
            try {
                ObjectInputStream in2 = new ObjectInputStream(inputStream);
                Object readObject = in2.readObject();
                try {
                    in2.close();
                } catch (IOException e) {
                }
                return readObject;
            } catch (ClassNotFoundException ex) {
                throw new SerializationException((Throwable) ex);
            } catch (IOException ex2) {
                throw new SerializationException((Throwable) ex2);
            } catch (Throwable th) {
                if (in != null) {
                    try {
                        in.close();
                    } catch (IOException e2) {
                    }
                }
                throw th;
            }
        } else {
            throw new IllegalArgumentException("The InputStream must not be null");
        }
    }

    public static Object deserialize(byte[] objectData) {
        if (objectData != null) {
            return deserialize((InputStream) new ByteArrayInputStream(objectData));
        }
        throw new IllegalArgumentException("The byte[] must not be null");
    }
}
