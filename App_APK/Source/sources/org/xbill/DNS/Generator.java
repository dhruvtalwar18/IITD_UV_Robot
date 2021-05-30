package org.xbill.DNS;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import org.apache.commons.httpclient.cookie.CookieSpec;
import sensor_msgs.NavSatStatus;

public class Generator {
    private long current;
    public final int dclass;
    public long end;
    public final String namePattern;
    public final Name origin;
    public final String rdataPattern;
    public long start;
    public long step;
    public final long ttl;
    public final int type;

    public static boolean supportedType(int type2) {
        Type.check(type2);
        return type2 == 12 || type2 == 5 || type2 == 39 || type2 == 1 || type2 == 28 || type2 == 2;
    }

    public Generator(long start2, long end2, long step2, String namePattern2, int type2, int dclass2, long ttl2, String rdataPattern2, Name origin2) {
        long j = start2;
        long j2 = end2;
        long j3 = step2;
        if (j < 0 || j2 < 0 || j > j2 || j3 <= 0) {
            String str = namePattern2;
            int i = type2;
            int i2 = dclass2;
            long j4 = ttl2;
            String str2 = rdataPattern2;
            Name name = origin2;
            throw new IllegalArgumentException("invalid range specification");
        } else if (supportedType(type2)) {
            DClass.check(dclass2);
            this.start = j;
            this.end = j2;
            this.step = j3;
            this.namePattern = namePattern2;
            this.type = type2;
            this.dclass = dclass2;
            this.ttl = ttl2;
            this.rdataPattern = rdataPattern2;
            this.origin = origin2;
            this.current = j;
        } else {
            String str3 = namePattern2;
            int i3 = type2;
            int i4 = dclass2;
            long j5 = ttl2;
            String str4 = rdataPattern2;
            Name name2 = origin2;
            throw new IllegalArgumentException("unsupported type");
        }
    }

    private String substitute(String spec, long n) throws IOException {
        byte[] str;
        String number;
        boolean escaped;
        long j;
        boolean escaped2 = false;
        byte[] str2 = spec.getBytes();
        StringBuffer sb = new StringBuffer();
        int i = 0;
        loop0:
        while (i < str2.length) {
            char c = (char) (str2[i] & NavSatStatus.STATUS_NO_FIX);
            if (escaped2) {
                sb.append(c);
                escaped2 = false;
            } else if (c != '\\') {
                if (c == '$') {
                    boolean negative = false;
                    long offset = 0;
                    long width = 0;
                    long base = 10;
                    boolean wantUpperCase = false;
                    if (i + 1 >= str2.length || str2[i + 1] != 36) {
                        if (i + 1 < str2.length && str2[i + 1] == 123) {
                            int i2 = i + 1;
                            if (i2 + 1 < str2.length && str2[i2 + 1] == 45) {
                                negative = true;
                                i2++;
                            }
                            while (i2 + 1 < str2.length) {
                                i2++;
                                c = (char) (str2[i2] & NavSatStatus.STATUS_NO_FIX);
                                if (c == ',' || c == '}') {
                                    break;
                                } else if (c < '0' || c > '9') {
                                    throw new TextParseException("invalid offset");
                                } else {
                                    c = (char) (c - '0');
                                    offset = (offset * 10) + ((long) c);
                                }
                            }
                            if (negative) {
                                offset = -offset;
                            }
                            if (c == ',') {
                                while (true) {
                                    if (i2 + 1 >= str2.length) {
                                        break;
                                    }
                                    i2++;
                                    c = (char) (str2[i2] & NavSatStatus.STATUS_NO_FIX);
                                    if (c == ',') {
                                        break;
                                    } else if (c == '}') {
                                        boolean z = negative;
                                        break;
                                    } else if (c < '0' || c > '9') {
                                    } else {
                                        c = (char) (c - '0');
                                        width = (width * 10) + ((long) c);
                                        negative = negative;
                                    }
                                }
                                throw new TextParseException("invalid width");
                            }
                            if (c == ',') {
                                if (i2 + 1 != str2.length) {
                                    i2++;
                                    char c2 = (char) (str2[i2] & NavSatStatus.STATUS_NO_FIX);
                                    if (c2 == 'o') {
                                        j = 8;
                                    } else if (c2 == 'x') {
                                        j = 16;
                                    } else if (c2 == 'X') {
                                        j = 16;
                                        wantUpperCase = true;
                                    } else if (c2 != 'd') {
                                        throw new TextParseException("invalid base");
                                    }
                                    base = j;
                                } else {
                                    throw new TextParseException("invalid base");
                                }
                            }
                            if (i2 + 1 == str2.length || str2[i2 + 1] != 125) {
                                throw new TextParseException("invalid modifiers");
                            }
                            i = i2 + 1;
                        }
                        long v = n + offset;
                        if (v >= 0) {
                            if (base == 8) {
                                number = Long.toOctalString(v);
                            } else if (base == 16) {
                                number = Long.toHexString(v);
                            } else {
                                number = Long.toString(v);
                            }
                            if (wantUpperCase) {
                                number = number.toUpperCase();
                            }
                            String number2 = number;
                            if (width != 0) {
                                escaped = escaped2;
                                str = str2;
                                if (width > ((long) number2.length())) {
                                    int zeros = ((int) width) - number2.length();
                                    while (true) {
                                        int zeros2 = zeros - 1;
                                        if (zeros <= 0) {
                                            break;
                                        }
                                        sb.append('0');
                                        zeros = zeros2;
                                    }
                                }
                            } else {
                                escaped = escaped2;
                                str = str2;
                            }
                            sb.append(number2);
                            escaped2 = escaped;
                        } else {
                            byte[] bArr = str2;
                            throw new TextParseException("invalid offset expansion");
                        }
                    } else {
                        i++;
                        sb.append((char) (str2[i] & NavSatStatus.STATUS_NO_FIX));
                    }
                } else {
                    str = str2;
                    sb.append(c);
                }
                i++;
                str2 = str;
            } else if (i + 1 != str2.length) {
                escaped2 = true;
            } else {
                throw new TextParseException("invalid escape character");
            }
            str = str2;
            i++;
            str2 = str;
        }
        boolean z2 = escaped2;
        byte[] bArr2 = str2;
        return sb.toString();
    }

    public Record nextRecord() throws IOException {
        if (this.current > this.end) {
            return null;
        }
        Name name = Name.fromString(substitute(this.namePattern, this.current), this.origin);
        String rdata = substitute(this.rdataPattern, this.current);
        this.current += this.step;
        return Record.fromString(name, this.type, this.dclass, this.ttl, rdata, this.origin);
    }

    public Record[] expand() throws IOException {
        List list = new ArrayList();
        long i = this.start;
        while (i < this.end) {
            Name name = Name.fromString(substitute(this.namePattern, this.current), this.origin);
            list.add(Record.fromString(name, this.type, this.dclass, this.ttl, substitute(this.rdataPattern, this.current), this.origin));
            i += this.step;
        }
        return (Record[]) list.toArray(new Record[list.size()]);
    }

    public String toString() {
        StringBuffer sb = new StringBuffer();
        sb.append("$GENERATE ");
        StringBuffer stringBuffer = new StringBuffer();
        stringBuffer.append(this.start);
        stringBuffer.append("-");
        stringBuffer.append(this.end);
        sb.append(stringBuffer.toString());
        if (this.step > 1) {
            StringBuffer stringBuffer2 = new StringBuffer();
            stringBuffer2.append(CookieSpec.PATH_DELIM);
            stringBuffer2.append(this.step);
            sb.append(stringBuffer2.toString());
        }
        sb.append(" ");
        StringBuffer stringBuffer3 = new StringBuffer();
        stringBuffer3.append(this.namePattern);
        stringBuffer3.append(" ");
        sb.append(stringBuffer3.toString());
        StringBuffer stringBuffer4 = new StringBuffer();
        stringBuffer4.append(this.ttl);
        stringBuffer4.append(" ");
        sb.append(stringBuffer4.toString());
        if (this.dclass != 1 || !Options.check("noPrintIN")) {
            StringBuffer stringBuffer5 = new StringBuffer();
            stringBuffer5.append(DClass.string(this.dclass));
            stringBuffer5.append(" ");
            sb.append(stringBuffer5.toString());
        }
        StringBuffer stringBuffer6 = new StringBuffer();
        stringBuffer6.append(Type.string(this.type));
        stringBuffer6.append(" ");
        sb.append(stringBuffer6.toString());
        StringBuffer stringBuffer7 = new StringBuffer();
        stringBuffer7.append(this.rdataPattern);
        stringBuffer7.append(" ");
        sb.append(stringBuffer7.toString());
        return sb.toString();
    }
}
