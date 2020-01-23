package org.letsbuildrockets.libs;

/**
 * VersionNumber
 */
public class VersionNumber {
    public int major = -1; 
    public int minor = -1;

    VersionNumber(int _major, int _minor) {
        major = _major;
        minor = _minor;
    }

    public boolean isOlderThan(VersionNumber req) {
        return(major < req.major || (major == req.major && minor < req.minor));
    }

    public String toString() {
        return String.format("%d.%d", major, minor);
    }
}