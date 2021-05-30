package org.apache.commons.io;

import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.io.OutputStream;
import java.io.Reader;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.StringTokenizer;
import rocon_std_msgs.Strings;

public class FileSystemUtils {
    private static final int INIT_PROBLEM = -1;
    private static final FileSystemUtils INSTANCE = new FileSystemUtils();
    private static final int OS;
    private static final int OTHER = 0;
    private static final int POSIX_UNIX = 3;
    private static final int UNIX = 2;
    private static final int WINDOWS = 1;

    static {
        int os;
        try {
            String osName = System.getProperty("os.name");
            if (osName != null) {
                String osName2 = osName.toLowerCase();
                if (osName2.indexOf("windows") != -1) {
                    os = 1;
                } else {
                    if (osName2.indexOf("linux") == -1 && osName2.indexOf("sun os") == -1 && osName2.indexOf("sunos") == -1 && osName2.indexOf("solaris") == -1 && osName2.indexOf("mpe/ix") == -1 && osName2.indexOf(Strings.OS_FREEBSD) == -1 && osName2.indexOf("irix") == -1 && osName2.indexOf("digital unix") == -1 && osName2.indexOf("unix") == -1) {
                        if (osName2.indexOf("mac os x") == -1) {
                            if (osName2.indexOf("hp-ux") == -1) {
                                if (osName2.indexOf("aix") == -1) {
                                    os = 0;
                                }
                            }
                            os = 3;
                        }
                    }
                    os = 2;
                }
                OS = os;
                return;
            }
            throw new IOException("os.name not found");
        } catch (Exception e) {
            os = -1;
        }
    }

    public static long freeSpace(String path) throws IOException {
        return INSTANCE.freeSpaceOS(path, OS, false);
    }

    public static long freeSpaceKb(String path) throws IOException {
        return INSTANCE.freeSpaceOS(path, OS, true);
    }

    /* access modifiers changed from: package-private */
    public long freeSpaceOS(String path, int os, boolean kb) throws IOException {
        if (path != null) {
            switch (os) {
                case 0:
                    throw new IllegalStateException("Unsupported operating system");
                case 1:
                    return kb ? freeSpaceWindows(path) / 1024 : freeSpaceWindows(path);
                case 2:
                    return freeSpaceUnix(path, kb, false);
                case 3:
                    return freeSpaceUnix(path, kb, true);
                default:
                    throw new IllegalStateException("Exception caught when determining operating system");
            }
        } else {
            throw new IllegalArgumentException("Path must not be empty");
        }
    }

    /* access modifiers changed from: package-private */
    public long freeSpaceWindows(String path) throws IOException {
        String path2 = FilenameUtils.normalize(path);
        if (path2.length() > 2 && path2.charAt(1) == ':') {
            path2 = path2.substring(0, 2);
        }
        StringBuffer stringBuffer = new StringBuffer();
        stringBuffer.append("dir /-c ");
        stringBuffer.append(path2);
        List lines = performCommand(new String[]{"cmd.exe", "/C", stringBuffer.toString()}, Integer.MAX_VALUE);
        int i = lines.size() - 1;
        while (true) {
            int i2 = i;
            if (i2 >= 0) {
                String line = (String) lines.get(i2);
                if (line.length() > 0) {
                    return parseDir(line, path2);
                }
                i = i2 - 1;
            } else {
                StringBuffer stringBuffer2 = new StringBuffer();
                stringBuffer2.append("Command line 'dir /-c' did not return any info for path '");
                stringBuffer2.append(path2);
                stringBuffer2.append("'");
                throw new IOException(stringBuffer2.toString());
            }
        }
    }

    /* access modifiers changed from: package-private */
    public long parseDir(String line, String path) throws IOException {
        int bytesStart = 0;
        int bytesEnd = 0;
        int j = line.length() - 1;
        while (true) {
            if (j < 0) {
                break;
            } else if (Character.isDigit(line.charAt(j))) {
                bytesEnd = j + 1;
                break;
            } else {
                j--;
            }
        }
        while (true) {
            if (j < 0) {
                break;
            }
            char c = line.charAt(j);
            if (!Character.isDigit(c) && c != ',' && c != '.') {
                bytesStart = j + 1;
                break;
            }
            j--;
        }
        if (j >= 0) {
            StringBuffer buf = new StringBuffer(line.substring(bytesStart, bytesEnd));
            int k = 0;
            while (k < buf.length()) {
                if (buf.charAt(k) == ',' || buf.charAt(k) == '.') {
                    buf.deleteCharAt(k);
                    k--;
                }
                k++;
            }
            return parseBytes(buf.toString(), path);
        }
        StringBuffer stringBuffer = new StringBuffer();
        stringBuffer.append("Command line 'dir /-c' did not return valid info for path '");
        stringBuffer.append(path);
        stringBuffer.append("'");
        throw new IOException(stringBuffer.toString());
    }

    /* access modifiers changed from: package-private */
    public long freeSpaceUnix(String path, boolean kb, boolean posix) throws IOException {
        if (path.length() != 0) {
            String path2 = FilenameUtils.normalize(path);
            String flags = "-";
            if (kb) {
                StringBuffer stringBuffer = new StringBuffer();
                stringBuffer.append(flags);
                stringBuffer.append("k");
                flags = stringBuffer.toString();
            }
            if (posix) {
                StringBuffer stringBuffer2 = new StringBuffer();
                stringBuffer2.append(flags);
                stringBuffer2.append("P");
                flags = stringBuffer2.toString();
            }
            List lines = performCommand(flags.length() > 1 ? new String[]{"df", flags, path2} : new String[]{"df", path2}, 3);
            if (lines.size() >= 2) {
                StringTokenizer tok = new StringTokenizer((String) lines.get(1), " ");
                if (tok.countTokens() >= 4) {
                    tok.nextToken();
                } else if (tok.countTokens() != 1 || lines.size() < 3) {
                    StringBuffer stringBuffer3 = new StringBuffer();
                    stringBuffer3.append("Command line 'df' did not return data as expected for path '");
                    stringBuffer3.append(path2);
                    stringBuffer3.append("'- check path is valid");
                    throw new IOException(stringBuffer3.toString());
                } else {
                    tok = new StringTokenizer((String) lines.get(2), " ");
                }
                tok.nextToken();
                tok.nextToken();
                return parseBytes(tok.nextToken(), path2);
            }
            StringBuffer stringBuffer4 = new StringBuffer();
            stringBuffer4.append("Command line 'df' did not return info as expected for path '");
            stringBuffer4.append(path2);
            stringBuffer4.append("'- response was ");
            stringBuffer4.append(lines);
            throw new IOException(stringBuffer4.toString());
        }
        throw new IllegalArgumentException("Path must not be empty");
    }

    /* access modifiers changed from: package-private */
    public long parseBytes(String freeSpace, String path) throws IOException {
        try {
            long bytes = Long.parseLong(freeSpace);
            if (bytes >= 0) {
                return bytes;
            }
            StringBuffer stringBuffer = new StringBuffer();
            stringBuffer.append("Command line 'df' did not find free space in response for path '");
            stringBuffer.append(path);
            stringBuffer.append("'- check path is valid");
            throw new IOException(stringBuffer.toString());
        } catch (NumberFormatException e) {
            StringBuffer stringBuffer2 = new StringBuffer();
            stringBuffer2.append("Command line 'df' did not return numeric data as expected for path '");
            stringBuffer2.append(path);
            stringBuffer2.append("'- check path is valid");
            throw new IOException(stringBuffer2.toString());
        }
    }

    /* access modifiers changed from: package-private */
    public List performCommand(String[] cmdAttribs, int max) throws IOException {
        List lines = new ArrayList(20);
        Process proc = null;
        InputStream in = null;
        OutputStream out = null;
        InputStream err = null;
        BufferedReader inr = null;
        try {
            proc = openProcess(cmdAttribs);
            in = proc.getInputStream();
            out = proc.getOutputStream();
            err = proc.getErrorStream();
            inr = new BufferedReader(new InputStreamReader(in));
            for (String line = inr.readLine(); line != null && lines.size() < max; line = inr.readLine()) {
                lines.add(line.toLowerCase().trim());
            }
            proc.waitFor();
            if (proc.exitValue() != 0) {
                StringBuffer stringBuffer = new StringBuffer();
                stringBuffer.append("Command line returned OS error code '");
                stringBuffer.append(proc.exitValue());
                stringBuffer.append("' for command ");
                stringBuffer.append(Arrays.asList(cmdAttribs));
                throw new IOException(stringBuffer.toString());
            } else if (lines.size() != 0) {
                IOUtils.closeQuietly(in);
                IOUtils.closeQuietly(out);
                IOUtils.closeQuietly(err);
                IOUtils.closeQuietly((Reader) inr);
                if (proc != null) {
                    proc.destroy();
                }
                return lines;
            } else {
                StringBuffer stringBuffer2 = new StringBuffer();
                stringBuffer2.append("Command line did not return any info for command ");
                stringBuffer2.append(Arrays.asList(cmdAttribs));
                throw new IOException(stringBuffer2.toString());
            }
        } catch (InterruptedException ex) {
            StringBuffer stringBuffer3 = new StringBuffer();
            stringBuffer3.append("Command line threw an InterruptedException '");
            stringBuffer3.append(ex.getMessage());
            stringBuffer3.append("' for command ");
            stringBuffer3.append(Arrays.asList(cmdAttribs));
            throw new IOException(stringBuffer3.toString());
        } catch (Throwable th) {
            IOUtils.closeQuietly(in);
            IOUtils.closeQuietly(out);
            IOUtils.closeQuietly(err);
            IOUtils.closeQuietly((Reader) inr);
            if (proc != null) {
                proc.destroy();
            }
            throw th;
        }
    }

    /* access modifiers changed from: package-private */
    public Process openProcess(String[] cmdAttribs) throws IOException {
        return Runtime.getRuntime().exec(cmdAttribs);
    }
}
