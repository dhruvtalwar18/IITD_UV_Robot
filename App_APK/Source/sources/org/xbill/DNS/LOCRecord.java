package org.xbill.DNS;

import java.io.IOException;
import java.text.DecimalFormat;
import java.text.NumberFormat;
import org.apache.commons.lang.time.DateUtils;
import org.bytedeco.javacpp.opencv_stitching;
import org.xbill.DNS.Tokenizer;

public class LOCRecord extends Record {
    private static final long serialVersionUID = 9058224788126750409L;
    private static NumberFormat w2 = new DecimalFormat();
    private static NumberFormat w3 = new DecimalFormat();
    private long altitude;
    private long hPrecision;
    private long latitude;
    private long longitude;
    private long size;
    private long vPrecision;

    static {
        w2.setMinimumIntegerDigits(2);
        w3.setMinimumIntegerDigits(3);
    }

    LOCRecord() {
    }

    /* access modifiers changed from: package-private */
    public Record getObject() {
        return new LOCRecord();
    }

    /* JADX INFO: super call moved to the top of the method (can break code semantics) */
    public LOCRecord(Name name, int dclass, long ttl, double latitude2, double longitude2, double altitude2, double size2, double hPrecision2, double vPrecision2) {
        super(name, 29, dclass, ttl);
        this.latitude = (long) ((latitude2 * 3600.0d * 1000.0d) + 2.147483648E9d);
        this.longitude = (long) ((3600.0d * longitude2 * 1000.0d) + 2.147483648E9d);
        this.altitude = (long) ((altitude2 + 100000.0d) * 100.0d);
        this.size = (long) (size2 * 100.0d);
        this.hPrecision = (long) (hPrecision2 * 100.0d);
        this.vPrecision = (long) (100.0d * vPrecision2);
    }

    /* access modifiers changed from: package-private */
    public void rrFromWire(DNSInput in) throws IOException {
        if (in.readU8() == 0) {
            this.size = parseLOCformat(in.readU8());
            this.hPrecision = parseLOCformat(in.readU8());
            this.vPrecision = parseLOCformat(in.readU8());
            this.latitude = in.readU32();
            this.longitude = in.readU32();
            this.altitude = in.readU32();
            return;
        }
        throw new WireParseException("Invalid LOC version");
    }

    private double parseFixedPoint(String s) {
        if (s.matches("^-?\\d+$")) {
            return (double) Integer.parseInt(s);
        }
        if (s.matches("^-?\\d+\\.\\d*$")) {
            String[] parts = s.split("\\.");
            double value = (double) Integer.parseInt(parts[0]);
            double fraction = (double) Integer.parseInt(parts[1]);
            if (value < opencv_stitching.Stitcher.ORIG_RESOL) {
                Double.isNaN(fraction);
                fraction *= -1.0d;
            }
            Double.isNaN(value);
            return (fraction / Math.pow(10.0d, (double) parts[1].length())) + value;
        }
        throw new NumberFormatException();
    }

    /* JADX WARNING: Removed duplicated region for block: B:29:0x008e  */
    /* JADX WARNING: Removed duplicated region for block: B:47:0x00e5  */
    /* Code decompiled incorrectly, please refer to instructions dump. */
    private long parsePosition(org.xbill.DNS.Tokenizer r19, java.lang.String r20) throws java.io.IOException {
        /*
            r18 = this;
            r1 = r19
            r2 = r20
            java.lang.String r0 = "latitude"
            boolean r3 = r2.equals(r0)
            r0 = 0
            r4 = 0
            r5 = 0
            int r7 = r19.getUInt16()
            r0 = 180(0xb4, float:2.52E-43)
            if (r7 > r0) goto L_0x00fb
            r0 = 90
            if (r7 <= r0) goto L_0x001c
            if (r3 != 0) goto L_0x00fb
        L_0x001c:
            java.lang.String r0 = r19.getString()
            r8 = r0
            int r0 = java.lang.Integer.parseInt(r8)     // Catch:{ NumberFormatException -> 0x0083 }
            r4 = r0
            if (r4 < 0) goto L_0x0064
            r0 = 59
            if (r4 > r0) goto L_0x0064
            java.lang.String r0 = r19.getString()     // Catch:{ NumberFormatException -> 0x0083 }
            r8 = r0
            r9 = r18
            double r10 = r9.parseFixedPoint(r8)     // Catch:{ NumberFormatException -> 0x0081 }
            r5 = r10
            r10 = 0
            int r0 = (r5 > r10 ? 1 : (r5 == r10 ? 0 : -1))
            if (r0 < 0) goto L_0x0049
            r10 = 4633641066610819072(0x404e000000000000, double:60.0)
            int r0 = (r5 > r10 ? 1 : (r5 == r10 ? 0 : -1))
            if (r0 >= 0) goto L_0x0049
            java.lang.String r0 = r19.getString()     // Catch:{ NumberFormatException -> 0x0081 }
            goto L_0x0087
        L_0x0049:
            java.lang.StringBuffer r0 = new java.lang.StringBuffer     // Catch:{ NumberFormatException -> 0x0081 }
            r0.<init>()     // Catch:{ NumberFormatException -> 0x0081 }
            java.lang.String r10 = "Invalid LOC "
            r0.append(r10)     // Catch:{ NumberFormatException -> 0x0081 }
            r0.append(r2)     // Catch:{ NumberFormatException -> 0x0081 }
            java.lang.String r10 = " seconds"
            r0.append(r10)     // Catch:{ NumberFormatException -> 0x0081 }
            java.lang.String r0 = r0.toString()     // Catch:{ NumberFormatException -> 0x0081 }
            org.xbill.DNS.TextParseException r0 = r1.exception(r0)     // Catch:{ NumberFormatException -> 0x0081 }
            throw r0     // Catch:{ NumberFormatException -> 0x0081 }
        L_0x0064:
            r9 = r18
            java.lang.StringBuffer r0 = new java.lang.StringBuffer     // Catch:{ NumberFormatException -> 0x0081 }
            r0.<init>()     // Catch:{ NumberFormatException -> 0x0081 }
            java.lang.String r10 = "Invalid LOC "
            r0.append(r10)     // Catch:{ NumberFormatException -> 0x0081 }
            r0.append(r2)     // Catch:{ NumberFormatException -> 0x0081 }
            java.lang.String r10 = " minutes"
            r0.append(r10)     // Catch:{ NumberFormatException -> 0x0081 }
            java.lang.String r0 = r0.toString()     // Catch:{ NumberFormatException -> 0x0081 }
            org.xbill.DNS.TextParseException r0 = r1.exception(r0)     // Catch:{ NumberFormatException -> 0x0081 }
            throw r0     // Catch:{ NumberFormatException -> 0x0081 }
        L_0x0081:
            r0 = move-exception
            goto L_0x0086
        L_0x0083:
            r0 = move-exception
            r9 = r18
        L_0x0086:
            r0 = r8
        L_0x0087:
            int r8 = r0.length()
            r10 = 1
            if (r8 != r10) goto L_0x00e5
            r10 = 4652007308841189376(0x408f400000000000, double:1000.0)
            long r12 = (long) r4
            long r14 = (long) r7
            r16 = 60
            long r14 = r14 * r16
            long r12 = r12 + r14
            long r12 = r12 * r16
            double r12 = (double) r12
            java.lang.Double.isNaN(r12)
            double r12 = r12 + r5
            double r12 = r12 * r10
            long r10 = (long) r12
            r8 = 0
            char r8 = r0.charAt(r8)
            char r8 = java.lang.Character.toUpperCase(r8)
            if (r3 == 0) goto L_0x00b3
            r12 = 83
            if (r8 == r12) goto L_0x00b9
        L_0x00b3:
            if (r3 != 0) goto L_0x00bb
            r12 = 87
            if (r8 != r12) goto L_0x00bb
        L_0x00b9:
            long r10 = -r10
            goto L_0x00de
        L_0x00bb:
            if (r3 == 0) goto L_0x00c1
            r12 = 78
            if (r8 != r12) goto L_0x00c8
        L_0x00c1:
            if (r3 != 0) goto L_0x00de
            r12 = 69
            if (r8 != r12) goto L_0x00c8
            goto L_0x00de
        L_0x00c8:
            java.lang.StringBuffer r12 = new java.lang.StringBuffer
            r12.<init>()
            java.lang.String r13 = "Invalid LOC "
            r12.append(r13)
            r12.append(r2)
            java.lang.String r12 = r12.toString()
            org.xbill.DNS.TextParseException r12 = r1.exception(r12)
            throw r12
        L_0x00de:
            r12 = 2147483648(0x80000000, double:1.0609978955E-314)
            long r10 = r10 + r12
            return r10
        L_0x00e5:
            java.lang.StringBuffer r8 = new java.lang.StringBuffer
            r8.<init>()
            java.lang.String r10 = "Invalid LOC "
            r8.append(r10)
            r8.append(r2)
            java.lang.String r8 = r8.toString()
            org.xbill.DNS.TextParseException r8 = r1.exception(r8)
            throw r8
        L_0x00fb:
            r9 = r18
            java.lang.StringBuffer r0 = new java.lang.StringBuffer
            r0.<init>()
            java.lang.String r8 = "Invalid LOC "
            r0.append(r8)
            r0.append(r2)
            java.lang.String r8 = " degrees"
            r0.append(r8)
            java.lang.String r0 = r0.toString()
            org.xbill.DNS.TextParseException r0 = r1.exception(r0)
            throw r0
        */
        throw new UnsupportedOperationException("Method not decompiled: org.xbill.DNS.LOCRecord.parsePosition(org.xbill.DNS.Tokenizer, java.lang.String):long");
    }

    private long parseDouble(Tokenizer st, String type, boolean required, long min, long max, long defaultValue) throws IOException {
        Tokenizer.Token token = st.get();
        if (!token.isEOL()) {
            String s = token.value;
            if (s.length() > 1 && s.charAt(s.length() - 1) == 'm') {
                s = s.substring(0, s.length() - 1);
            }
            try {
                long value = (long) (parseFixedPoint(s) * 100.0d);
                if (value >= min && value <= max) {
                    return value;
                }
                StringBuffer stringBuffer = new StringBuffer();
                stringBuffer.append("Invalid LOC ");
                stringBuffer.append(type);
                throw st.exception(stringBuffer.toString());
            } catch (NumberFormatException e) {
                StringBuffer stringBuffer2 = new StringBuffer();
                stringBuffer2.append("Invalid LOC ");
                stringBuffer2.append(type);
                throw st.exception(stringBuffer2.toString());
            }
        } else if (!required) {
            st.unget();
            return defaultValue;
        } else {
            StringBuffer stringBuffer3 = new StringBuffer();
            stringBuffer3.append("Invalid LOC ");
            stringBuffer3.append(type);
            throw st.exception(stringBuffer3.toString());
        }
    }

    /* access modifiers changed from: package-private */
    public void rdataFromString(Tokenizer st, Name origin) throws IOException {
        this.latitude = parsePosition(st, "latitude");
        this.longitude = parsePosition(st, "longitude");
        this.altitude = parseDouble(st, "altitude", true, -10000000, 4284967295L, 0) + 10000000;
        Tokenizer tokenizer = st;
        this.size = parseDouble(tokenizer, "size", false, 0, 9000000000L, 100);
        this.hPrecision = parseDouble(tokenizer, "horizontal precision", false, 0, 9000000000L, 1000000);
        this.vPrecision = parseDouble(tokenizer, "vertical precision", false, 0, 9000000000L, 1000);
    }

    private void renderFixedPoint(StringBuffer sb, NumberFormat formatter, long value, long divisor) {
        sb.append(value / divisor);
        long value2 = value % divisor;
        if (value2 != 0) {
            sb.append(".");
            sb.append(formatter.format(value2));
        }
    }

    private String positionToString(long value, char pos, char neg) {
        char direction;
        StringBuffer sb = new StringBuffer();
        long temp = value - 2147483648L;
        if (temp < 0) {
            temp = -temp;
            direction = neg;
        } else {
            direction = pos;
        }
        sb.append(temp / DateUtils.MILLIS_PER_HOUR);
        long temp2 = temp % DateUtils.MILLIS_PER_HOUR;
        sb.append(" ");
        sb.append(temp2 / DateUtils.MILLIS_PER_MINUTE);
        long temp3 = temp2 % DateUtils.MILLIS_PER_MINUTE;
        sb.append(" ");
        renderFixedPoint(sb, w3, temp3, 1000);
        sb.append(" ");
        sb.append(direction);
        return sb.toString();
    }

    /* access modifiers changed from: package-private */
    public String rrToString() {
        StringBuffer sb = new StringBuffer();
        sb.append(positionToString(this.latitude, 'N', 'S'));
        sb.append(" ");
        sb.append(positionToString(this.longitude, 'E', 'W'));
        sb.append(" ");
        StringBuffer stringBuffer = sb;
        renderFixedPoint(stringBuffer, w2, this.altitude - 10000000, 100);
        sb.append("m ");
        renderFixedPoint(stringBuffer, w2, this.size, 100);
        sb.append("m ");
        renderFixedPoint(stringBuffer, w2, this.hPrecision, 100);
        sb.append("m ");
        renderFixedPoint(stringBuffer, w2, this.vPrecision, 100);
        sb.append("m");
        return sb.toString();
    }

    public double getLatitude() {
        double d = (double) (this.latitude - 2147483648L);
        Double.isNaN(d);
        return d / 3600000.0d;
    }

    public double getLongitude() {
        double d = (double) (this.longitude - 2147483648L);
        Double.isNaN(d);
        return d / 3600000.0d;
    }

    public double getAltitude() {
        double d = (double) (this.altitude - 10000000);
        Double.isNaN(d);
        return d / 100.0d;
    }

    public double getSize() {
        double d = (double) this.size;
        Double.isNaN(d);
        return d / 100.0d;
    }

    public double getHPrecision() {
        double d = (double) this.hPrecision;
        Double.isNaN(d);
        return d / 100.0d;
    }

    public double getVPrecision() {
        double d = (double) this.vPrecision;
        Double.isNaN(d);
        return d / 100.0d;
    }

    /* access modifiers changed from: package-private */
    public void rrToWire(DNSOutput out, Compression c, boolean canonical) {
        out.writeU8(0);
        out.writeU8(toLOCformat(this.size));
        out.writeU8(toLOCformat(this.hPrecision));
        out.writeU8(toLOCformat(this.vPrecision));
        out.writeU32(this.latitude);
        out.writeU32(this.longitude);
        out.writeU32(this.altitude);
    }

    private static long parseLOCformat(int b) throws WireParseException {
        long out = (long) (b >> 4);
        int exp = b & 15;
        if (out > 9 || exp > 9) {
            throw new WireParseException("Invalid LOC Encoding");
        }
        while (true) {
            int exp2 = exp - 1;
            if (exp <= 0) {
                return out;
            }
            out *= 10;
            exp = exp2;
        }
    }

    private int toLOCformat(long l) {
        byte exp = 0;
        while (l > 9) {
            exp = (byte) (exp + 1);
            l /= 10;
        }
        return (int) ((l << 4) + ((long) exp));
    }
}
