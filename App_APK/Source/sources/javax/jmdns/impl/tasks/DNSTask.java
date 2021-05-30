package javax.jmdns.impl.tasks;

import java.io.IOException;
import java.util.Timer;
import java.util.TimerTask;
import javax.jmdns.impl.DNSIncoming;
import javax.jmdns.impl.DNSOutgoing;
import javax.jmdns.impl.DNSQuestion;
import javax.jmdns.impl.DNSRecord;
import javax.jmdns.impl.JmDNSImpl;

public abstract class DNSTask extends TimerTask {
    private final JmDNSImpl _jmDNSImpl;

    public abstract String getName();

    public abstract void start(Timer timer);

    protected DNSTask(JmDNSImpl jmDNSImpl) {
        this._jmDNSImpl = jmDNSImpl;
    }

    public JmDNSImpl getDns() {
        return this._jmDNSImpl;
    }

    public String toString() {
        return getName();
    }

    public DNSOutgoing addQuestion(DNSOutgoing out, DNSQuestion rec) throws IOException {
        DNSOutgoing newOut = out;
        try {
            newOut.addQuestion(rec);
            return newOut;
        } catch (IOException e) {
            int flags = newOut.getFlags();
            boolean multicast = newOut.isMulticast();
            int maxUDPPayload = newOut.getMaxUDPPayload();
            int id = newOut.getId();
            newOut.setFlags(flags | 512);
            newOut.setId(id);
            this._jmDNSImpl.send(newOut);
            DNSOutgoing newOut2 = new DNSOutgoing(flags, multicast, maxUDPPayload);
            newOut2.addQuestion(rec);
            return newOut2;
        }
    }

    public DNSOutgoing addAnswer(DNSOutgoing out, DNSIncoming in, DNSRecord rec) throws IOException {
        DNSOutgoing newOut = out;
        try {
            newOut.addAnswer(in, rec);
            return newOut;
        } catch (IOException e) {
            int flags = newOut.getFlags();
            boolean multicast = newOut.isMulticast();
            int maxUDPPayload = newOut.getMaxUDPPayload();
            int id = newOut.getId();
            newOut.setFlags(flags | 512);
            newOut.setId(id);
            this._jmDNSImpl.send(newOut);
            DNSOutgoing newOut2 = new DNSOutgoing(flags, multicast, maxUDPPayload);
            newOut2.addAnswer(in, rec);
            return newOut2;
        }
    }

    public DNSOutgoing addAnswer(DNSOutgoing out, DNSRecord rec, long now) throws IOException {
        DNSOutgoing newOut = out;
        try {
            newOut.addAnswer(rec, now);
            return newOut;
        } catch (IOException e) {
            int flags = newOut.getFlags();
            boolean multicast = newOut.isMulticast();
            int maxUDPPayload = newOut.getMaxUDPPayload();
            int id = newOut.getId();
            newOut.setFlags(flags | 512);
            newOut.setId(id);
            this._jmDNSImpl.send(newOut);
            DNSOutgoing newOut2 = new DNSOutgoing(flags, multicast, maxUDPPayload);
            newOut2.addAnswer(rec, now);
            return newOut2;
        }
    }

    public DNSOutgoing addAuthoritativeAnswer(DNSOutgoing out, DNSRecord rec) throws IOException {
        DNSOutgoing newOut = out;
        try {
            newOut.addAuthorativeAnswer(rec);
            return newOut;
        } catch (IOException e) {
            int flags = newOut.getFlags();
            boolean multicast = newOut.isMulticast();
            int maxUDPPayload = newOut.getMaxUDPPayload();
            int id = newOut.getId();
            newOut.setFlags(flags | 512);
            newOut.setId(id);
            this._jmDNSImpl.send(newOut);
            DNSOutgoing newOut2 = new DNSOutgoing(flags, multicast, maxUDPPayload);
            newOut2.addAuthorativeAnswer(rec);
            return newOut2;
        }
    }

    public DNSOutgoing addAdditionalAnswer(DNSOutgoing out, DNSIncoming in, DNSRecord rec) throws IOException {
        DNSOutgoing newOut = out;
        try {
            newOut.addAdditionalAnswer(in, rec);
            return newOut;
        } catch (IOException e) {
            int flags = newOut.getFlags();
            boolean multicast = newOut.isMulticast();
            int maxUDPPayload = newOut.getMaxUDPPayload();
            int id = newOut.getId();
            newOut.setFlags(flags | 512);
            newOut.setId(id);
            this._jmDNSImpl.send(newOut);
            DNSOutgoing newOut2 = new DNSOutgoing(flags, multicast, maxUDPPayload);
            newOut2.addAdditionalAnswer(in, rec);
            return newOut2;
        }
    }
}
