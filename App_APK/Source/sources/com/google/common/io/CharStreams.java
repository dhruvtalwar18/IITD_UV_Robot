package com.google.common.io;

import com.google.common.annotations.Beta;
import com.google.common.base.Preconditions;
import java.io.Closeable;
import java.io.EOFException;
import java.io.IOException;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.io.OutputStream;
import java.io.OutputStreamWriter;
import java.io.Reader;
import java.io.StringReader;
import java.io.Writer;
import java.nio.CharBuffer;
import java.nio.charset.Charset;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

@Beta
public final class CharStreams {
    private static final int BUF_SIZE = 2048;

    private CharStreams() {
    }

    public static InputSupplier<StringReader> newReaderSupplier(final String value) {
        Preconditions.checkNotNull(value);
        return new InputSupplier<StringReader>() {
            public StringReader getInput() {
                return new StringReader(value);
            }
        };
    }

    public static InputSupplier<InputStreamReader> newReaderSupplier(final InputSupplier<? extends InputStream> in, final Charset charset) {
        Preconditions.checkNotNull(in);
        Preconditions.checkNotNull(charset);
        return new InputSupplier<InputStreamReader>() {
            public InputStreamReader getInput() throws IOException {
                return new InputStreamReader((InputStream) in.getInput(), charset);
            }
        };
    }

    public static OutputSupplier<OutputStreamWriter> newWriterSupplier(final OutputSupplier<? extends OutputStream> out, final Charset charset) {
        Preconditions.checkNotNull(out);
        Preconditions.checkNotNull(charset);
        return new OutputSupplier<OutputStreamWriter>() {
            public OutputStreamWriter getOutput() throws IOException {
                return new OutputStreamWriter((OutputStream) out.getOutput(), charset);
            }
        };
    }

    public static <W extends Appendable & Closeable> void write(CharSequence from, OutputSupplier<W> to) throws IOException {
        Preconditions.checkNotNull(from);
        boolean threw = true;
        W out = (Appendable) to.getOutput();
        try {
            out.append(from);
            threw = false;
        } finally {
            Closeables.close((Closeable) out, threw);
        }
    }

    /* JADX INFO: finally extract failed */
    public static <R extends Readable & Closeable, W extends Appendable & Closeable> long copy(InputSupplier<R> from, OutputSupplier<W> to) throws IOException {
        W out;
        int successfulOps = 0;
        R in = (Readable) from.getInput();
        boolean z = false;
        try {
            out = (Appendable) to.getOutput();
            long count = copy((Readable) in, (Appendable) out);
            successfulOps = 0 + 1;
            Closeables.close((Closeable) out, successfulOps < 1);
            Closeable closeable = (Closeable) in;
            if (successfulOps + 1 < 2) {
                z = true;
            }
            Closeables.close(closeable, z);
            return count;
        } catch (Throwable out2) {
            Closeable closeable2 = (Closeable) in;
            if (successfulOps < 2) {
                z = true;
            }
            Closeables.close(closeable2, z);
            throw out2;
        }
    }

    public static <R extends Readable & Closeable> long copy(InputSupplier<R> from, Appendable to) throws IOException {
        boolean threw = true;
        R in = (Readable) from.getInput();
        try {
            threw = false;
            return copy((Readable) in, to);
        } finally {
            Closeables.close((Closeable) in, threw);
        }
    }

    public static long copy(Readable from, Appendable to) throws IOException {
        CharBuffer buf = CharBuffer.allocate(2048);
        long total = 0;
        while (true) {
            int r = from.read(buf);
            if (r == -1) {
                return total;
            }
            buf.flip();
            to.append(buf, 0, r);
            total += (long) r;
        }
    }

    public static String toString(Readable r) throws IOException {
        return toStringBuilder(r).toString();
    }

    public static <R extends Readable & Closeable> String toString(InputSupplier<R> supplier) throws IOException {
        return toStringBuilder(supplier).toString();
    }

    private static StringBuilder toStringBuilder(Readable r) throws IOException {
        StringBuilder sb = new StringBuilder();
        copy(r, (Appendable) sb);
        return sb;
    }

    private static <R extends Readable & Closeable> StringBuilder toStringBuilder(InputSupplier<R> supplier) throws IOException {
        boolean threw = true;
        R r = (Readable) supplier.getInput();
        try {
            threw = false;
            return toStringBuilder((Readable) r);
        } finally {
            Closeables.close((Closeable) r, threw);
        }
    }

    public static <R extends Readable & Closeable> String readFirstLine(InputSupplier<R> supplier) throws IOException {
        boolean threw = true;
        R r = (Readable) supplier.getInput();
        try {
            threw = false;
            return new LineReader(r).readLine();
        } finally {
            Closeables.close((Closeable) r, threw);
        }
    }

    public static <R extends Readable & Closeable> List<String> readLines(InputSupplier<R> supplier) throws IOException {
        boolean threw = true;
        R r = (Readable) supplier.getInput();
        try {
            threw = false;
            return readLines((Readable) r);
        } finally {
            Closeables.close((Closeable) r, threw);
        }
    }

    public static List<String> readLines(Readable r) throws IOException {
        List<String> result = new ArrayList<>();
        LineReader lineReader = new LineReader(r);
        while (true) {
            String readLine = lineReader.readLine();
            String line = readLine;
            if (readLine == null) {
                return result;
            }
            result.add(line);
        }
    }

    /* JADX INFO: finally extract failed */
    public static <R extends Readable & Closeable, T> T readLines(InputSupplier<R> supplier, LineProcessor<T> callback) throws IOException {
        String line;
        R r = (Readable) supplier.getInput();
        try {
            LineReader lineReader = new LineReader(r);
            do {
                String readLine = lineReader.readLine();
                line = readLine;
                if (readLine == null || !callback.processLine(line)) {
                    Closeables.close((Closeable) r, false);
                }
                String readLine2 = lineReader.readLine();
                line = readLine2;
                break;
            } while (!callback.processLine(line));
            Closeables.close((Closeable) r, false);
            return callback.getResult();
        } catch (Throwable th) {
            Closeables.close((Closeable) r, true);
            throw th;
        }
    }

    public static InputSupplier<Reader> join(final Iterable<? extends InputSupplier<? extends Reader>> suppliers) {
        return new InputSupplier<Reader>() {
            public Reader getInput() throws IOException {
                return new MultiReader(suppliers.iterator());
            }
        };
    }

    public static InputSupplier<Reader> join(InputSupplier<? extends Reader>... suppliers) {
        return join((Iterable<? extends InputSupplier<? extends Reader>>) Arrays.asList(suppliers));
    }

    public static void skipFully(Reader reader, long n) throws IOException {
        while (n > 0) {
            long amt = reader.skip(n);
            if (amt != 0) {
                n -= amt;
            } else if (reader.read() != -1) {
                n--;
            } else {
                throw new EOFException();
            }
        }
    }

    public static Writer asWriter(Appendable target) {
        if (target instanceof Writer) {
            return (Writer) target;
        }
        return new AppendableWriter(target);
    }
}
