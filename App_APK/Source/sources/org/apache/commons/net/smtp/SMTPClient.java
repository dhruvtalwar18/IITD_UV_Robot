package org.apache.commons.net.smtp;

import java.io.IOException;
import java.io.Writer;
import org.apache.commons.net.io.DotTerminatedMessageWriter;

public class SMTPClient extends SMTP {
    public SMTPClient() {
    }

    public SMTPClient(String encoding) {
        super(encoding);
    }

    public boolean completePendingCommand() throws IOException {
        return SMTPReply.isPositiveCompletion(getReply());
    }

    public boolean login(String hostname) throws IOException {
        return SMTPReply.isPositiveCompletion(helo(hostname));
    }

    public boolean login() throws IOException {
        String name = getLocalAddress().getHostName();
        if (name == null) {
            return false;
        }
        return SMTPReply.isPositiveCompletion(helo(name));
    }

    public boolean setSender(RelayPath path) throws IOException {
        return SMTPReply.isPositiveCompletion(mail(path.toString()));
    }

    public boolean setSender(String address) throws IOException {
        return SMTPReply.isPositiveCompletion(mail("<" + address + ">"));
    }

    public boolean addRecipient(RelayPath path) throws IOException {
        return SMTPReply.isPositiveCompletion(rcpt(path.toString()));
    }

    public boolean addRecipient(String address) throws IOException {
        return SMTPReply.isPositiveCompletion(rcpt("<" + address + ">"));
    }

    public Writer sendMessageData() throws IOException {
        if (!SMTPReply.isPositiveIntermediate(data())) {
            return null;
        }
        return new DotTerminatedMessageWriter(this._writer);
    }

    public boolean sendShortMessageData(String message) throws IOException {
        Writer writer = sendMessageData();
        if (writer == null) {
            return false;
        }
        writer.write(message);
        writer.close();
        return completePendingCommand();
    }

    public boolean sendSimpleMessage(String sender, String recipient, String message) throws IOException {
        if (setSender(sender) && addRecipient(recipient)) {
            return sendShortMessageData(message);
        }
        return false;
    }

    public boolean sendSimpleMessage(String sender, String[] recipients, String message) throws IOException {
        if (!setSender(sender)) {
            return false;
        }
        boolean oneSuccess = false;
        for (String addRecipient : recipients) {
            if (addRecipient(addRecipient)) {
                oneSuccess = true;
            }
        }
        if (!oneSuccess) {
            return false;
        }
        return sendShortMessageData(message);
    }

    public boolean logout() throws IOException {
        return SMTPReply.isPositiveCompletion(quit());
    }

    public boolean reset() throws IOException {
        return SMTPReply.isPositiveCompletion(rset());
    }

    public boolean verify(String username) throws IOException {
        int result = vrfy(username);
        return result == 250 || result == 251;
    }

    public String listHelp() throws IOException {
        if (SMTPReply.isPositiveCompletion(help())) {
            return getReplyString();
        }
        return null;
    }

    public String listHelp(String command) throws IOException {
        if (SMTPReply.isPositiveCompletion(help(command))) {
            return getReplyString();
        }
        return null;
    }

    public boolean sendNoOp() throws IOException {
        return SMTPReply.isPositiveCompletion(noop());
    }
}
