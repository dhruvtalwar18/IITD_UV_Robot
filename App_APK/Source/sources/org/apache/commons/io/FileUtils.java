package org.apache.commons.io;

import java.io.File;
import java.io.FileFilter;
import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;
import java.net.URL;
import java.util.ArrayList;
import java.util.Collection;
import java.util.Date;
import java.util.Iterator;
import java.util.LinkedList;
import java.util.List;
import java.util.zip.CRC32;
import java.util.zip.CheckedInputStream;
import java.util.zip.Checksum;
import org.apache.commons.io.filefilter.DirectoryFileFilter;
import org.apache.commons.io.filefilter.FalseFileFilter;
import org.apache.commons.io.filefilter.FileFilterUtils;
import org.apache.commons.io.filefilter.IOFileFilter;
import org.apache.commons.io.filefilter.SuffixFileFilter;
import org.apache.commons.io.filefilter.TrueFileFilter;
import org.apache.commons.io.output.NullOutputStream;

public class FileUtils {
    public static final File[] EMPTY_FILE_ARRAY = new File[0];
    public static final long ONE_GB = 1073741824;
    public static final long ONE_KB = 1024;
    public static final long ONE_MB = 1048576;

    public static FileInputStream openInputStream(File file) throws IOException {
        if (!file.exists()) {
            StringBuffer stringBuffer = new StringBuffer();
            stringBuffer.append("File '");
            stringBuffer.append(file);
            stringBuffer.append("' does not exist");
            throw new FileNotFoundException(stringBuffer.toString());
        } else if (file.isDirectory()) {
            StringBuffer stringBuffer2 = new StringBuffer();
            stringBuffer2.append("File '");
            stringBuffer2.append(file);
            stringBuffer2.append("' exists but is a directory");
            throw new IOException(stringBuffer2.toString());
        } else if (file.canRead()) {
            return new FileInputStream(file);
        } else {
            StringBuffer stringBuffer3 = new StringBuffer();
            stringBuffer3.append("File '");
            stringBuffer3.append(file);
            stringBuffer3.append("' cannot be read");
            throw new IOException(stringBuffer3.toString());
        }
    }

    public static FileOutputStream openOutputStream(File file) throws IOException {
        if (!file.exists()) {
            File parent = file.getParentFile();
            if (parent != null && !parent.exists() && !parent.mkdirs()) {
                StringBuffer stringBuffer = new StringBuffer();
                stringBuffer.append("File '");
                stringBuffer.append(file);
                stringBuffer.append("' could not be created");
                throw new IOException(stringBuffer.toString());
            }
        } else if (file.isDirectory()) {
            StringBuffer stringBuffer2 = new StringBuffer();
            stringBuffer2.append("File '");
            stringBuffer2.append(file);
            stringBuffer2.append("' exists but is a directory");
            throw new IOException(stringBuffer2.toString());
        } else if (!file.canWrite()) {
            StringBuffer stringBuffer3 = new StringBuffer();
            stringBuffer3.append("File '");
            stringBuffer3.append(file);
            stringBuffer3.append("' cannot be written to");
            throw new IOException(stringBuffer3.toString());
        }
        return new FileOutputStream(file);
    }

    public static String byteCountToDisplaySize(long size) {
        if (size / ONE_GB > 0) {
            StringBuffer stringBuffer = new StringBuffer();
            stringBuffer.append(String.valueOf(size / ONE_GB));
            stringBuffer.append(" GB");
            return stringBuffer.toString();
        } else if (size / 1048576 > 0) {
            StringBuffer stringBuffer2 = new StringBuffer();
            stringBuffer2.append(String.valueOf(size / 1048576));
            stringBuffer2.append(" MB");
            return stringBuffer2.toString();
        } else if (size / 1024 > 0) {
            StringBuffer stringBuffer3 = new StringBuffer();
            stringBuffer3.append(String.valueOf(size / 1024));
            stringBuffer3.append(" KB");
            return stringBuffer3.toString();
        } else {
            StringBuffer stringBuffer4 = new StringBuffer();
            stringBuffer4.append(String.valueOf(size));
            stringBuffer4.append(" bytes");
            return stringBuffer4.toString();
        }
    }

    public static void touch(File file) throws IOException {
        if (!file.exists()) {
            IOUtils.closeQuietly(openOutputStream(file));
        }
        if (!file.setLastModified(System.currentTimeMillis())) {
            StringBuffer stringBuffer = new StringBuffer();
            stringBuffer.append("Unable to set the last modification time for ");
            stringBuffer.append(file);
            throw new IOException(stringBuffer.toString());
        }
    }

    public static File[] convertFileCollectionToFileArray(Collection files) {
        return (File[]) files.toArray(new File[files.size()]);
    }

    private static void innerListFiles(Collection files, File directory, IOFileFilter filter) {
        File[] found = directory.listFiles(filter);
        if (found != null) {
            for (int i = 0; i < found.length; i++) {
                if (found[i].isDirectory()) {
                    innerListFiles(files, found[i], filter);
                } else {
                    files.add(found[i]);
                }
            }
        }
    }

    public static Collection listFiles(File directory, IOFileFilter fileFilter, IOFileFilter dirFilter) {
        IOFileFilter effDirFilter;
        if (!directory.isDirectory()) {
            throw new IllegalArgumentException("Parameter 'directory' is not a directory");
        } else if (fileFilter != null) {
            IOFileFilter effFileFilter = FileFilterUtils.andFileFilter(fileFilter, FileFilterUtils.notFileFilter(DirectoryFileFilter.INSTANCE));
            if (dirFilter == null) {
                effDirFilter = FalseFileFilter.INSTANCE;
            } else {
                effDirFilter = FileFilterUtils.andFileFilter(dirFilter, DirectoryFileFilter.INSTANCE);
            }
            Collection files = new LinkedList();
            innerListFiles(files, directory, FileFilterUtils.orFileFilter(effFileFilter, effDirFilter));
            return files;
        } else {
            throw new NullPointerException("Parameter 'fileFilter' is null");
        }
    }

    public static Iterator iterateFiles(File directory, IOFileFilter fileFilter, IOFileFilter dirFilter) {
        return listFiles(directory, fileFilter, dirFilter).iterator();
    }

    private static String[] toSuffixes(String[] extensions) {
        String[] suffixes = new String[extensions.length];
        for (int i = 0; i < extensions.length; i++) {
            StringBuffer stringBuffer = new StringBuffer();
            stringBuffer.append(".");
            stringBuffer.append(extensions[i]);
            suffixes[i] = stringBuffer.toString();
        }
        return suffixes;
    }

    public static Collection listFiles(File directory, String[] extensions, boolean recursive) {
        IOFileFilter filter;
        if (extensions == null) {
            filter = TrueFileFilter.INSTANCE;
        } else {
            filter = new SuffixFileFilter(toSuffixes(extensions));
        }
        return listFiles(directory, filter, recursive ? TrueFileFilter.INSTANCE : FalseFileFilter.INSTANCE);
    }

    public static Iterator iterateFiles(File directory, String[] extensions, boolean recursive) {
        return listFiles(directory, extensions, recursive).iterator();
    }

    public static boolean contentEquals(File file1, File file2) throws IOException {
        boolean file1Exists = file1.exists();
        if (file1Exists != file2.exists()) {
            return false;
        }
        if (!file1Exists) {
            return true;
        }
        if (file1.isDirectory() || file2.isDirectory()) {
            throw new IOException("Can't compare directories, only files");
        } else if (file1.length() != file2.length()) {
            return false;
        } else {
            if (file1.getCanonicalFile().equals(file2.getCanonicalFile())) {
                return true;
            }
            InputStream input1 = null;
            InputStream input2 = null;
            try {
                input1 = new FileInputStream(file1);
                input2 = new FileInputStream(file2);
                return IOUtils.contentEquals(input1, input2);
            } finally {
                IOUtils.closeQuietly(input1);
                IOUtils.closeQuietly(input2);
            }
        }
    }

    public static File toFile(URL url) {
        if (url == null || !url.getProtocol().equals(HttpPostBodyUtil.FILE)) {
            return null;
        }
        String filename = url.getFile().replace(IOUtils.DIR_SEPARATOR_UNIX, File.separatorChar);
        int pos = 0;
        while (true) {
            int indexOf = filename.indexOf(37, pos);
            pos = indexOf;
            if (indexOf < 0) {
                return new File(filename);
            }
            if (pos + 2 < filename.length()) {
                StringBuffer stringBuffer = new StringBuffer();
                stringBuffer.append(filename.substring(0, pos));
                stringBuffer.append((char) Integer.parseInt(filename.substring(pos + 1, pos + 3), 16));
                stringBuffer.append(filename.substring(pos + 3));
                filename = stringBuffer.toString();
            }
        }
    }

    public static File[] toFiles(URL[] urls) {
        if (urls == null || urls.length == 0) {
            return EMPTY_FILE_ARRAY;
        }
        File[] files = new File[urls.length];
        for (int i = 0; i < urls.length; i++) {
            URL url = urls[i];
            if (url != null) {
                if (url.getProtocol().equals(HttpPostBodyUtil.FILE)) {
                    files[i] = toFile(url);
                } else {
                    StringBuffer stringBuffer = new StringBuffer();
                    stringBuffer.append("URL could not be converted to a File: ");
                    stringBuffer.append(url);
                    throw new IllegalArgumentException(stringBuffer.toString());
                }
            }
        }
        return files;
    }

    public static URL[] toURLs(File[] files) throws IOException {
        URL[] urls = new URL[files.length];
        for (int i = 0; i < urls.length; i++) {
            urls[i] = files[i].toURL();
        }
        return urls;
    }

    public static void copyFileToDirectory(File srcFile, File destDir) throws IOException {
        copyFileToDirectory(srcFile, destDir, true);
    }

    public static void copyFileToDirectory(File srcFile, File destDir, boolean preserveFileDate) throws IOException {
        if (destDir == null) {
            throw new NullPointerException("Destination must not be null");
        } else if (!destDir.exists() || destDir.isDirectory()) {
            copyFile(srcFile, new File(destDir, srcFile.getName()), preserveFileDate);
        } else {
            StringBuffer stringBuffer = new StringBuffer();
            stringBuffer.append("Destination '");
            stringBuffer.append(destDir);
            stringBuffer.append("' is not a directory");
            throw new IllegalArgumentException(stringBuffer.toString());
        }
    }

    public static void copyFile(File srcFile, File destFile) throws IOException {
        copyFile(srcFile, destFile, true);
    }

    public static void copyFile(File srcFile, File destFile, boolean preserveFileDate) throws IOException {
        if (srcFile == null) {
            throw new NullPointerException("Source must not be null");
        } else if (destFile == null) {
            throw new NullPointerException("Destination must not be null");
        } else if (!srcFile.exists()) {
            StringBuffer stringBuffer = new StringBuffer();
            stringBuffer.append("Source '");
            stringBuffer.append(srcFile);
            stringBuffer.append("' does not exist");
            throw new FileNotFoundException(stringBuffer.toString());
        } else if (srcFile.isDirectory()) {
            StringBuffer stringBuffer2 = new StringBuffer();
            stringBuffer2.append("Source '");
            stringBuffer2.append(srcFile);
            stringBuffer2.append("' exists but is a directory");
            throw new IOException(stringBuffer2.toString());
        } else if (srcFile.getCanonicalPath().equals(destFile.getCanonicalPath())) {
            StringBuffer stringBuffer3 = new StringBuffer();
            stringBuffer3.append("Source '");
            stringBuffer3.append(srcFile);
            stringBuffer3.append("' and destination '");
            stringBuffer3.append(destFile);
            stringBuffer3.append("' are the same");
            throw new IOException(stringBuffer3.toString());
        } else if (destFile.getParentFile() != null && !destFile.getParentFile().exists() && !destFile.getParentFile().mkdirs()) {
            StringBuffer stringBuffer4 = new StringBuffer();
            stringBuffer4.append("Destination '");
            stringBuffer4.append(destFile);
            stringBuffer4.append("' directory cannot be created");
            throw new IOException(stringBuffer4.toString());
        } else if (!destFile.exists() || destFile.canWrite()) {
            doCopyFile(srcFile, destFile, preserveFileDate);
        } else {
            StringBuffer stringBuffer5 = new StringBuffer();
            stringBuffer5.append("Destination '");
            stringBuffer5.append(destFile);
            stringBuffer5.append("' exists but is read-only");
            throw new IOException(stringBuffer5.toString());
        }
    }

    private static void doCopyFile(File srcFile, File destFile, boolean preserveFileDate) throws IOException {
        FileOutputStream output;
        if (!destFile.exists() || !destFile.isDirectory()) {
            FileInputStream input = new FileInputStream(srcFile);
            try {
                output = new FileOutputStream(destFile);
                IOUtils.copy((InputStream) input, (OutputStream) output);
                IOUtils.closeQuietly((OutputStream) output);
                IOUtils.closeQuietly((InputStream) input);
                if (srcFile.length() != destFile.length()) {
                    StringBuffer stringBuffer = new StringBuffer();
                    stringBuffer.append("Failed to copy full contents from '");
                    stringBuffer.append(srcFile);
                    stringBuffer.append("' to '");
                    stringBuffer.append(destFile);
                    stringBuffer.append("'");
                    throw new IOException(stringBuffer.toString());
                } else if (preserveFileDate) {
                    destFile.setLastModified(srcFile.lastModified());
                }
            } catch (Throwable th) {
                IOUtils.closeQuietly((InputStream) input);
                throw th;
            }
        } else {
            StringBuffer stringBuffer2 = new StringBuffer();
            stringBuffer2.append("Destination '");
            stringBuffer2.append(destFile);
            stringBuffer2.append("' exists but is a directory");
            throw new IOException(stringBuffer2.toString());
        }
    }

    public static void copyDirectoryToDirectory(File srcDir, File destDir) throws IOException {
        if (srcDir == null) {
            throw new NullPointerException("Source must not be null");
        } else if (srcDir.exists() && !srcDir.isDirectory()) {
            StringBuffer stringBuffer = new StringBuffer();
            stringBuffer.append("Source '");
            stringBuffer.append(destDir);
            stringBuffer.append("' is not a directory");
            throw new IllegalArgumentException(stringBuffer.toString());
        } else if (destDir == null) {
            throw new NullPointerException("Destination must not be null");
        } else if (!destDir.exists() || destDir.isDirectory()) {
            copyDirectory(srcDir, new File(destDir, srcDir.getName()), true);
        } else {
            StringBuffer stringBuffer2 = new StringBuffer();
            stringBuffer2.append("Destination '");
            stringBuffer2.append(destDir);
            stringBuffer2.append("' is not a directory");
            throw new IllegalArgumentException(stringBuffer2.toString());
        }
    }

    public static void copyDirectory(File srcDir, File destDir) throws IOException {
        copyDirectory(srcDir, destDir, true);
    }

    public static void copyDirectory(File srcDir, File destDir, boolean preserveFileDate) throws IOException {
        copyDirectory(srcDir, destDir, (FileFilter) null, preserveFileDate);
    }

    public static void copyDirectory(File srcDir, File destDir, FileFilter filter) throws IOException {
        copyDirectory(srcDir, destDir, filter, true);
    }

    public static void copyDirectory(File srcDir, File destDir, FileFilter filter, boolean preserveFileDate) throws IOException {
        if (srcDir == null) {
            throw new NullPointerException("Source must not be null");
        } else if (destDir == null) {
            throw new NullPointerException("Destination must not be null");
        } else if (!srcDir.exists()) {
            StringBuffer stringBuffer = new StringBuffer();
            stringBuffer.append("Source '");
            stringBuffer.append(srcDir);
            stringBuffer.append("' does not exist");
            throw new FileNotFoundException(stringBuffer.toString());
        } else if (!srcDir.isDirectory()) {
            StringBuffer stringBuffer2 = new StringBuffer();
            stringBuffer2.append("Source '");
            stringBuffer2.append(srcDir);
            stringBuffer2.append("' exists but is not a directory");
            throw new IOException(stringBuffer2.toString());
        } else if (!srcDir.getCanonicalPath().equals(destDir.getCanonicalPath())) {
            List exclusionList = null;
            if (destDir.getCanonicalPath().startsWith(srcDir.getCanonicalPath())) {
                File[] srcFiles = filter == null ? srcDir.listFiles() : srcDir.listFiles(filter);
                if (srcFiles != null && srcFiles.length > 0) {
                    exclusionList = new ArrayList(srcFiles.length);
                    for (File name : srcFiles) {
                        exclusionList.add(new File(destDir, name.getName()).getCanonicalPath());
                    }
                }
            }
            doCopyDirectory(srcDir, destDir, filter, preserveFileDate, exclusionList);
        } else {
            StringBuffer stringBuffer3 = new StringBuffer();
            stringBuffer3.append("Source '");
            stringBuffer3.append(srcDir);
            stringBuffer3.append("' and destination '");
            stringBuffer3.append(destDir);
            stringBuffer3.append("' are the same");
            throw new IOException(stringBuffer3.toString());
        }
    }

    private static void doCopyDirectory(File srcDir, File destDir, FileFilter filter, boolean preserveFileDate, List exclusionList) throws IOException {
        if (destDir.exists()) {
            if (!destDir.isDirectory()) {
                StringBuffer stringBuffer = new StringBuffer();
                stringBuffer.append("Destination '");
                stringBuffer.append(destDir);
                stringBuffer.append("' exists but is not a directory");
                throw new IOException(stringBuffer.toString());
            }
        } else if (!destDir.mkdirs()) {
            StringBuffer stringBuffer2 = new StringBuffer();
            stringBuffer2.append("Destination '");
            stringBuffer2.append(destDir);
            stringBuffer2.append("' directory cannot be created");
            throw new IOException(stringBuffer2.toString());
        } else if (preserveFileDate) {
            destDir.setLastModified(srcDir.lastModified());
        }
        if (destDir.canWrite()) {
            File[] files = filter == null ? srcDir.listFiles() : srcDir.listFiles(filter);
            if (files != null) {
                for (int i = 0; i < files.length; i++) {
                    File copiedFile = new File(destDir, files[i].getName());
                    if (exclusionList == null || !exclusionList.contains(files[i].getCanonicalPath())) {
                        if (files[i].isDirectory()) {
                            doCopyDirectory(files[i], copiedFile, filter, preserveFileDate, exclusionList);
                        } else {
                            doCopyFile(files[i], copiedFile, preserveFileDate);
                        }
                    }
                }
                return;
            }
            StringBuffer stringBuffer3 = new StringBuffer();
            stringBuffer3.append("Failed to list contents of ");
            stringBuffer3.append(srcDir);
            throw new IOException(stringBuffer3.toString());
        }
        StringBuffer stringBuffer4 = new StringBuffer();
        stringBuffer4.append("Destination '");
        stringBuffer4.append(destDir);
        stringBuffer4.append("' cannot be written to");
        throw new IOException(stringBuffer4.toString());
    }

    public static void copyURLToFile(URL source, File destination) throws IOException {
        FileOutputStream output;
        InputStream input = source.openStream();
        try {
            output = openOutputStream(destination);
            IOUtils.copy(input, (OutputStream) output);
            IOUtils.closeQuietly((OutputStream) output);
            IOUtils.closeQuietly(input);
        } catch (Throwable th) {
            IOUtils.closeQuietly(input);
            throw th;
        }
    }

    public static void deleteDirectory(File directory) throws IOException {
        if (directory.exists()) {
            cleanDirectory(directory);
            if (!directory.delete()) {
                StringBuffer stringBuffer = new StringBuffer();
                stringBuffer.append("Unable to delete directory ");
                stringBuffer.append(directory);
                stringBuffer.append(".");
                throw new IOException(stringBuffer.toString());
            }
        }
    }

    public static boolean deleteQuietly(File file) {
        if (file == null) {
            return false;
        }
        try {
            if (file.isDirectory()) {
                cleanDirectory(file);
            }
        } catch (Exception e) {
        }
        try {
            return file.delete();
        } catch (Exception e2) {
            return false;
        }
    }

    public static void cleanDirectory(File directory) throws IOException {
        if (!directory.exists()) {
            StringBuffer stringBuffer = new StringBuffer();
            stringBuffer.append(directory);
            stringBuffer.append(" does not exist");
            throw new IllegalArgumentException(stringBuffer.toString());
        } else if (directory.isDirectory()) {
            File[] files = directory.listFiles();
            if (files != null) {
                IOException exception = null;
                for (File file : files) {
                    try {
                        forceDelete(file);
                    } catch (IOException ioe) {
                        exception = ioe;
                    }
                }
                if (exception != null) {
                    throw exception;
                }
                return;
            }
            StringBuffer stringBuffer2 = new StringBuffer();
            stringBuffer2.append("Failed to list contents of ");
            stringBuffer2.append(directory);
            throw new IOException(stringBuffer2.toString());
        } else {
            StringBuffer stringBuffer3 = new StringBuffer();
            stringBuffer3.append(directory);
            stringBuffer3.append(" is not a directory");
            throw new IllegalArgumentException(stringBuffer3.toString());
        }
    }

    public static boolean waitFor(File file, int seconds) {
        int timeout = 0;
        int tick = 0;
        while (!file.exists()) {
            int tick2 = tick + 1;
            if (tick >= 10) {
                tick = 0;
                int timeout2 = timeout + 1;
                if (timeout > seconds) {
                    return false;
                }
                timeout = timeout2;
            } else {
                tick = tick2;
            }
            try {
                Thread.sleep(100);
            } catch (InterruptedException e) {
            } catch (Exception e2) {
                return true;
            }
        }
        return true;
    }

    public static String readFileToString(File file, String encoding) throws IOException {
        InputStream in = null;
        try {
            in = openInputStream(file);
            return IOUtils.toString(in, encoding);
        } finally {
            IOUtils.closeQuietly(in);
        }
    }

    public static String readFileToString(File file) throws IOException {
        return readFileToString(file, (String) null);
    }

    public static byte[] readFileToByteArray(File file) throws IOException {
        InputStream in = null;
        try {
            in = openInputStream(file);
            return IOUtils.toByteArray(in);
        } finally {
            IOUtils.closeQuietly(in);
        }
    }

    public static List readLines(File file, String encoding) throws IOException {
        InputStream in = null;
        try {
            in = openInputStream(file);
            return IOUtils.readLines(in, encoding);
        } finally {
            IOUtils.closeQuietly(in);
        }
    }

    public static List readLines(File file) throws IOException {
        return readLines(file, (String) null);
    }

    public static LineIterator lineIterator(File file, String encoding) throws IOException {
        try {
            return IOUtils.lineIterator(openInputStream(file), encoding);
        } catch (IOException ex) {
            IOUtils.closeQuietly((InputStream) null);
            throw ex;
        } catch (RuntimeException ex2) {
            IOUtils.closeQuietly((InputStream) null);
            throw ex2;
        }
    }

    public static LineIterator lineIterator(File file) throws IOException {
        return lineIterator(file, (String) null);
    }

    public static void writeStringToFile(File file, String data, String encoding) throws IOException {
        OutputStream out = null;
        try {
            out = openOutputStream(file);
            IOUtils.write(data, out, encoding);
        } finally {
            IOUtils.closeQuietly(out);
        }
    }

    public static void writeStringToFile(File file, String data) throws IOException {
        writeStringToFile(file, data, (String) null);
    }

    public static void writeByteArrayToFile(File file, byte[] data) throws IOException {
        OutputStream out = null;
        try {
            out = openOutputStream(file);
            out.write(data);
        } finally {
            IOUtils.closeQuietly(out);
        }
    }

    public static void writeLines(File file, String encoding, Collection lines) throws IOException {
        writeLines(file, encoding, lines, (String) null);
    }

    public static void writeLines(File file, Collection lines) throws IOException {
        writeLines(file, (String) null, lines, (String) null);
    }

    public static void writeLines(File file, String encoding, Collection lines, String lineEnding) throws IOException {
        OutputStream out = null;
        try {
            out = openOutputStream(file);
            IOUtils.writeLines(lines, lineEnding, out, encoding);
        } finally {
            IOUtils.closeQuietly(out);
        }
    }

    public static void writeLines(File file, Collection lines, String lineEnding) throws IOException {
        writeLines(file, (String) null, lines, lineEnding);
    }

    public static void forceDelete(File file) throws IOException {
        if (file.isDirectory()) {
            deleteDirectory(file);
            return;
        }
        boolean filePresent = file.exists();
        if (file.delete()) {
            return;
        }
        if (!filePresent) {
            StringBuffer stringBuffer = new StringBuffer();
            stringBuffer.append("File does not exist: ");
            stringBuffer.append(file);
            throw new FileNotFoundException(stringBuffer.toString());
        }
        StringBuffer stringBuffer2 = new StringBuffer();
        stringBuffer2.append("Unable to delete file: ");
        stringBuffer2.append(file);
        throw new IOException(stringBuffer2.toString());
    }

    public static void forceDeleteOnExit(File file) throws IOException {
        if (file.isDirectory()) {
            deleteDirectoryOnExit(file);
        } else {
            file.deleteOnExit();
        }
    }

    private static void deleteDirectoryOnExit(File directory) throws IOException {
        if (directory.exists()) {
            cleanDirectoryOnExit(directory);
            directory.deleteOnExit();
        }
    }

    private static void cleanDirectoryOnExit(File directory) throws IOException {
        if (!directory.exists()) {
            StringBuffer stringBuffer = new StringBuffer();
            stringBuffer.append(directory);
            stringBuffer.append(" does not exist");
            throw new IllegalArgumentException(stringBuffer.toString());
        } else if (directory.isDirectory()) {
            File[] files = directory.listFiles();
            if (files != null) {
                IOException exception = null;
                for (File file : files) {
                    try {
                        forceDeleteOnExit(file);
                    } catch (IOException ioe) {
                        exception = ioe;
                    }
                }
                if (exception != null) {
                    throw exception;
                }
                return;
            }
            StringBuffer stringBuffer2 = new StringBuffer();
            stringBuffer2.append("Failed to list contents of ");
            stringBuffer2.append(directory);
            throw new IOException(stringBuffer2.toString());
        } else {
            StringBuffer stringBuffer3 = new StringBuffer();
            stringBuffer3.append(directory);
            stringBuffer3.append(" is not a directory");
            throw new IllegalArgumentException(stringBuffer3.toString());
        }
    }

    public static void forceMkdir(File directory) throws IOException {
        if (directory.exists()) {
            if (directory.isFile()) {
                StringBuffer stringBuffer = new StringBuffer();
                stringBuffer.append("File ");
                stringBuffer.append(directory);
                stringBuffer.append(" exists and is ");
                stringBuffer.append("not a directory. Unable to create directory.");
                throw new IOException(stringBuffer.toString());
            }
        } else if (!directory.mkdirs()) {
            StringBuffer stringBuffer2 = new StringBuffer();
            stringBuffer2.append("Unable to create directory ");
            stringBuffer2.append(directory);
            throw new IOException(stringBuffer2.toString());
        }
    }

    public static long sizeOfDirectory(File directory) {
        long j;
        if (!directory.exists()) {
            StringBuffer stringBuffer = new StringBuffer();
            stringBuffer.append(directory);
            stringBuffer.append(" does not exist");
            throw new IllegalArgumentException(stringBuffer.toString());
        } else if (directory.isDirectory()) {
            long size = 0;
            File[] files = directory.listFiles();
            if (files == null) {
                return 0;
            }
            for (File file : files) {
                if (file.isDirectory()) {
                    j = sizeOfDirectory(file);
                } else {
                    j = file.length();
                }
                size += j;
            }
            return size;
        } else {
            StringBuffer stringBuffer2 = new StringBuffer();
            stringBuffer2.append(directory);
            stringBuffer2.append(" is not a directory");
            throw new IllegalArgumentException(stringBuffer2.toString());
        }
    }

    public static boolean isFileNewer(File file, File reference) {
        if (reference == null) {
            throw new IllegalArgumentException("No specified reference file");
        } else if (reference.exists()) {
            return isFileNewer(file, reference.lastModified());
        } else {
            StringBuffer stringBuffer = new StringBuffer();
            stringBuffer.append("The reference file '");
            stringBuffer.append(file);
            stringBuffer.append("' doesn't exist");
            throw new IllegalArgumentException(stringBuffer.toString());
        }
    }

    public static boolean isFileNewer(File file, Date date) {
        if (date != null) {
            return isFileNewer(file, date.getTime());
        }
        throw new IllegalArgumentException("No specified date");
    }

    public static boolean isFileNewer(File file, long timeMillis) {
        if (file == null) {
            throw new IllegalArgumentException("No specified file");
        } else if (file.exists() && file.lastModified() > timeMillis) {
            return true;
        } else {
            return false;
        }
    }

    public static boolean isFileOlder(File file, File reference) {
        if (reference == null) {
            throw new IllegalArgumentException("No specified reference file");
        } else if (reference.exists()) {
            return isFileOlder(file, reference.lastModified());
        } else {
            StringBuffer stringBuffer = new StringBuffer();
            stringBuffer.append("The reference file '");
            stringBuffer.append(file);
            stringBuffer.append("' doesn't exist");
            throw new IllegalArgumentException(stringBuffer.toString());
        }
    }

    public static boolean isFileOlder(File file, Date date) {
        if (date != null) {
            return isFileOlder(file, date.getTime());
        }
        throw new IllegalArgumentException("No specified date");
    }

    public static boolean isFileOlder(File file, long timeMillis) {
        if (file == null) {
            throw new IllegalArgumentException("No specified file");
        } else if (file.exists() && file.lastModified() < timeMillis) {
            return true;
        } else {
            return false;
        }
    }

    public static long checksumCRC32(File file) throws IOException {
        CRC32 crc = new CRC32();
        checksum(file, crc);
        return crc.getValue();
    }

    public static Checksum checksum(File file, Checksum checksum) throws IOException {
        if (!file.isDirectory()) {
            InputStream in = null;
            try {
                in = new CheckedInputStream(new FileInputStream(file), checksum);
                IOUtils.copy(in, (OutputStream) new NullOutputStream());
                return checksum;
            } finally {
                IOUtils.closeQuietly(in);
            }
        } else {
            throw new IllegalArgumentException("Checksums can't be computed on directories");
        }
    }

    public static void moveDirectory(File srcDir, File destDir) throws IOException {
        if (srcDir == null) {
            throw new NullPointerException("Source must not be null");
        } else if (destDir == null) {
            throw new NullPointerException("Destination must not be null");
        } else if (!srcDir.exists()) {
            StringBuffer stringBuffer = new StringBuffer();
            stringBuffer.append("Source '");
            stringBuffer.append(srcDir);
            stringBuffer.append("' does not exist");
            throw new FileNotFoundException(stringBuffer.toString());
        } else if (!srcDir.isDirectory()) {
            StringBuffer stringBuffer2 = new StringBuffer();
            stringBuffer2.append("Source '");
            stringBuffer2.append(srcDir);
            stringBuffer2.append("' is not a directory");
            throw new IOException(stringBuffer2.toString());
        } else if (destDir.exists()) {
            StringBuffer stringBuffer3 = new StringBuffer();
            stringBuffer3.append("Destination '");
            stringBuffer3.append(destDir);
            stringBuffer3.append("' already exists");
            throw new IOException(stringBuffer3.toString());
        } else if (!srcDir.renameTo(destDir)) {
            copyDirectory(srcDir, destDir);
            deleteDirectory(srcDir);
            if (srcDir.exists()) {
                StringBuffer stringBuffer4 = new StringBuffer();
                stringBuffer4.append("Failed to delete original directory '");
                stringBuffer4.append(srcDir);
                stringBuffer4.append("' after copy to '");
                stringBuffer4.append(destDir);
                stringBuffer4.append("'");
                throw new IOException(stringBuffer4.toString());
            }
        }
    }

    public static void moveDirectoryToDirectory(File src, File destDir, boolean createDestDir) throws IOException {
        if (src == null) {
            throw new NullPointerException("Source must not be null");
        } else if (destDir != null) {
            if (!destDir.exists() && createDestDir) {
                destDir.mkdirs();
            }
            if (!destDir.exists()) {
                StringBuffer stringBuffer = new StringBuffer();
                stringBuffer.append("Destination directory '");
                stringBuffer.append(destDir);
                stringBuffer.append("' does not exist [createDestDir=");
                stringBuffer.append(createDestDir);
                stringBuffer.append("]");
                throw new FileNotFoundException(stringBuffer.toString());
            } else if (destDir.isDirectory()) {
                moveDirectory(src, new File(destDir, src.getName()));
            } else {
                StringBuffer stringBuffer2 = new StringBuffer();
                stringBuffer2.append("Destination '");
                stringBuffer2.append(destDir);
                stringBuffer2.append("' is not a directory");
                throw new IOException(stringBuffer2.toString());
            }
        } else {
            throw new NullPointerException("Destination directory must not be null");
        }
    }

    public static void moveFile(File srcFile, File destFile) throws IOException {
        if (srcFile == null) {
            throw new NullPointerException("Source must not be null");
        } else if (destFile == null) {
            throw new NullPointerException("Destination must not be null");
        } else if (!srcFile.exists()) {
            StringBuffer stringBuffer = new StringBuffer();
            stringBuffer.append("Source '");
            stringBuffer.append(srcFile);
            stringBuffer.append("' does not exist");
            throw new FileNotFoundException(stringBuffer.toString());
        } else if (srcFile.isDirectory()) {
            StringBuffer stringBuffer2 = new StringBuffer();
            stringBuffer2.append("Source '");
            stringBuffer2.append(srcFile);
            stringBuffer2.append("' is a directory");
            throw new IOException(stringBuffer2.toString());
        } else if (destFile.exists()) {
            StringBuffer stringBuffer3 = new StringBuffer();
            stringBuffer3.append("Destination '");
            stringBuffer3.append(destFile);
            stringBuffer3.append("' already exists");
            throw new IOException(stringBuffer3.toString());
        } else if (destFile.isDirectory()) {
            StringBuffer stringBuffer4 = new StringBuffer();
            stringBuffer4.append("Destination '");
            stringBuffer4.append(destFile);
            stringBuffer4.append("' is a directory");
            throw new IOException(stringBuffer4.toString());
        } else if (!srcFile.renameTo(destFile)) {
            copyFile(srcFile, destFile);
            if (!srcFile.delete()) {
                deleteQuietly(destFile);
                StringBuffer stringBuffer5 = new StringBuffer();
                stringBuffer5.append("Failed to delete original file '");
                stringBuffer5.append(srcFile);
                stringBuffer5.append("' after copy to '");
                stringBuffer5.append(destFile);
                stringBuffer5.append("'");
                throw new IOException(stringBuffer5.toString());
            }
        }
    }

    public static void moveFileToDirectory(File srcFile, File destDir, boolean createDestDir) throws IOException {
        if (srcFile == null) {
            throw new NullPointerException("Source must not be null");
        } else if (destDir != null) {
            if (!destDir.exists() && createDestDir) {
                destDir.mkdirs();
            }
            if (!destDir.exists()) {
                StringBuffer stringBuffer = new StringBuffer();
                stringBuffer.append("Destination directory '");
                stringBuffer.append(destDir);
                stringBuffer.append("' does not exist [createDestDir=");
                stringBuffer.append(createDestDir);
                stringBuffer.append("]");
                throw new FileNotFoundException(stringBuffer.toString());
            } else if (destDir.isDirectory()) {
                moveFile(srcFile, new File(destDir, srcFile.getName()));
            } else {
                StringBuffer stringBuffer2 = new StringBuffer();
                stringBuffer2.append("Destination '");
                stringBuffer2.append(destDir);
                stringBuffer2.append("' is not a directory");
                throw new IOException(stringBuffer2.toString());
            }
        } else {
            throw new NullPointerException("Destination directory must not be null");
        }
    }

    public static void moveToDirectory(File src, File destDir, boolean createDestDir) throws IOException {
        if (src == null) {
            throw new NullPointerException("Source must not be null");
        } else if (destDir == null) {
            throw new NullPointerException("Destination must not be null");
        } else if (!src.exists()) {
            StringBuffer stringBuffer = new StringBuffer();
            stringBuffer.append("Source '");
            stringBuffer.append(src);
            stringBuffer.append("' does not exist");
            throw new FileNotFoundException(stringBuffer.toString());
        } else if (src.isDirectory()) {
            moveDirectoryToDirectory(src, destDir, createDestDir);
        } else {
            moveFileToDirectory(src, destDir, createDestDir);
        }
    }
}
