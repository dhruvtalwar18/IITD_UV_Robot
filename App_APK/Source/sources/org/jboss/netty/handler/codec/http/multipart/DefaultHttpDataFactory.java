package org.jboss.netty.handler.codec.http.multipart;

import android.support.v4.media.session.PlaybackStateCompat;
import java.io.IOException;
import java.nio.charset.Charset;
import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.ConcurrentHashMap;
import org.jboss.netty.handler.codec.http.HttpRequest;

public class DefaultHttpDataFactory implements HttpDataFactory {
    public static long MINSIZE = PlaybackStateCompat.ACTION_PREPARE;
    private final boolean checkSize;
    private long minSize;
    private final ConcurrentHashMap<HttpRequest, List<HttpData>> requestFileDeleteMap;
    private final boolean useDisk;

    public DefaultHttpDataFactory() {
        this.requestFileDeleteMap = new ConcurrentHashMap<>();
        this.useDisk = false;
        this.checkSize = true;
        this.minSize = MINSIZE;
    }

    public DefaultHttpDataFactory(boolean useDisk2) {
        this.requestFileDeleteMap = new ConcurrentHashMap<>();
        this.useDisk = useDisk2;
        this.checkSize = false;
    }

    public DefaultHttpDataFactory(long minSize2) {
        this.requestFileDeleteMap = new ConcurrentHashMap<>();
        this.useDisk = false;
        this.checkSize = true;
        this.minSize = minSize2;
    }

    private List<HttpData> getList(HttpRequest request) {
        List<HttpData> list = this.requestFileDeleteMap.get(request);
        if (list != null) {
            return list;
        }
        List<HttpData> list2 = new ArrayList<>();
        this.requestFileDeleteMap.put(request, list2);
        return list2;
    }

    public Attribute createAttribute(HttpRequest request, String name) {
        if (this.useDisk) {
            Attribute attribute = new DiskAttribute(name);
            getList(request).add(attribute);
            return attribute;
        } else if (!this.checkSize) {
            return new MemoryAttribute(name);
        } else {
            Attribute attribute2 = new MixedAttribute(name, this.minSize);
            getList(request).add(attribute2);
            return attribute2;
        }
    }

    public Attribute createAttribute(HttpRequest request, String name, String value) {
        Attribute attribute;
        if (this.useDisk) {
            try {
                attribute = new DiskAttribute(name, value);
            } catch (IOException e) {
                attribute = new MixedAttribute(name, value, this.minSize);
            }
            getList(request).add(attribute);
            return attribute;
        } else if (this.checkSize) {
            Attribute attribute2 = new MixedAttribute(name, value, this.minSize);
            getList(request).add(attribute2);
            return attribute2;
        } else {
            try {
                return new MemoryAttribute(name, value);
            } catch (IOException e2) {
                throw new IllegalArgumentException(e2);
            }
        }
    }

    public FileUpload createFileUpload(HttpRequest request, String name, String filename, String contentType, String contentTransferEncoding, Charset charset, long size) {
        if (this.useDisk) {
            DiskFileUpload diskFileUpload = new DiskFileUpload(name, filename, contentType, contentTransferEncoding, charset, size);
            getList(request).add(diskFileUpload);
            return diskFileUpload;
        } else if (!this.checkSize) {
            return new MemoryFileUpload(name, filename, contentType, contentTransferEncoding, charset, size);
        } else {
            MixedFileUpload mixedFileUpload = new MixedFileUpload(name, filename, contentType, contentTransferEncoding, charset, size, this.minSize);
            getList(request).add(mixedFileUpload);
            return mixedFileUpload;
        }
    }

    public void removeHttpDataFromClean(HttpRequest request, InterfaceHttpData data) {
        if (data instanceof HttpData) {
            getList(request).remove(data);
        }
    }

    public void cleanRequestHttpDatas(HttpRequest request) {
        List<HttpData> fileToDelete = this.requestFileDeleteMap.remove(request);
        if (fileToDelete != null) {
            for (HttpData data : fileToDelete) {
                data.delete();
            }
            fileToDelete.clear();
        }
    }

    public void cleanAllHttpDatas() {
        for (HttpRequest request : this.requestFileDeleteMap.keySet()) {
            List<HttpData> fileToDelete = this.requestFileDeleteMap.get(request);
            if (fileToDelete != null) {
                for (HttpData data : fileToDelete) {
                    data.delete();
                }
                fileToDelete.clear();
            }
            this.requestFileDeleteMap.remove(request);
        }
    }
}
