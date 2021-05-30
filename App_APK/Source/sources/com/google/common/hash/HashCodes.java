package com.google.common.hash;

import com.google.common.annotations.Beta;
import com.google.common.base.Ascii;
import com.google.common.base.Preconditions;
import java.io.Serializable;
import sensor_msgs.NavSatStatus;

@Beta
public final class HashCodes {
    private HashCodes() {
    }

    public static HashCode fromInt(int hash) {
        return new IntHashCode(hash);
    }

    private static final class IntHashCode extends HashCode implements Serializable {
        private static final long serialVersionUID = 0;
        final int hash;

        IntHashCode(int hash2) {
            this.hash = hash2;
        }

        public int bits() {
            return 32;
        }

        public byte[] asBytes() {
            return new byte[]{(byte) this.hash, (byte) (this.hash >> 8), (byte) (this.hash >> 16), (byte) (this.hash >> 24)};
        }

        public int asInt() {
            return this.hash;
        }

        public long asLong() {
            throw new IllegalStateException("this HashCode only has 32 bits; cannot create a long");
        }
    }

    public static HashCode fromLong(long hash) {
        return new LongHashCode(hash);
    }

    private static final class LongHashCode extends HashCode implements Serializable {
        private static final long serialVersionUID = 0;
        final long hash;

        LongHashCode(long hash2) {
            this.hash = hash2;
        }

        public int bits() {
            return 64;
        }

        public byte[] asBytes() {
            return new byte[]{(byte) ((int) this.hash), (byte) ((int) (this.hash >> 8)), (byte) ((int) (this.hash >> 16)), (byte) ((int) (this.hash >> 24)), (byte) ((int) (this.hash >> 32)), (byte) ((int) (this.hash >> 40)), (byte) ((int) (this.hash >> 48)), (byte) ((int) (this.hash >> 56))};
        }

        public int asInt() {
            return (int) this.hash;
        }

        public long asLong() {
            return this.hash;
        }
    }

    public static HashCode fromBytes(byte[] bytes) {
        Preconditions.checkArgument(bytes.length >= 4, "A HashCode must contain at least 4 bytes.");
        return fromBytesNoCopy((byte[]) bytes.clone());
    }

    static HashCode fromBytesNoCopy(byte[] bytes) {
        return new BytesHashCode(bytes);
    }

    private static final class BytesHashCode extends HashCode implements Serializable {
        private static final long serialVersionUID = 0;
        final byte[] bytes;

        BytesHashCode(byte[] bytes2) {
            this.bytes = bytes2;
        }

        public int bits() {
            return this.bytes.length * 8;
        }

        public byte[] asBytes() {
            return (byte[]) this.bytes.clone();
        }

        public int asInt() {
            return (this.bytes[0] & NavSatStatus.STATUS_NO_FIX) | ((this.bytes[1] & NavSatStatus.STATUS_NO_FIX) << 8) | ((this.bytes[2] & NavSatStatus.STATUS_NO_FIX) << 16) | ((this.bytes[3] & NavSatStatus.STATUS_NO_FIX) << Ascii.CAN);
        }

        public long asLong() {
            if (this.bytes.length >= 8) {
                return ((((long) this.bytes[1]) & 255) << 8) | (((long) this.bytes[0]) & 255) | ((((long) this.bytes[2]) & 255) << 16) | ((((long) this.bytes[3]) & 255) << 24) | ((((long) this.bytes[4]) & 255) << 32) | ((((long) this.bytes[5]) & 255) << 40) | ((((long) this.bytes[6]) & 255) << 48) | ((((long) this.bytes[7]) & 255) << 56);
            }
            throw new IllegalStateException("Not enough bytes");
        }
    }
}
