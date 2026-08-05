"""Generate include/wifi_credentials.h from the environment.

Keeps WiFi credentials out of the repository: the header is gitignored and is
rewritten on every build from WIFI_CLAITEC_SSID and WIFI_CLAITEC_PASS.

Writing a header rather than passing -D flags avoids shell quoting entirely, so
credentials containing spaces, quotes or non-ASCII characters work unchanged.
"""

import os
import pathlib
import re

Import("env")  # noqa: F821 - injected by PlatformIO


def c_string_literal(value):
    """Render `value` as a C string literal, escaping whatever needs it."""
    out = []
    for ch in value:
        if ch in ('\\', '"'):
            out.append('\\' + ch)
        elif ch == '\n':
            out.append('\\n')
        elif ch == '\r':
            out.append('\\r')
        elif ch == '\t':
            out.append('\\t')
        elif 0x20 <= ord(ch) < 0x7F:
            out.append(ch)
        else:
            # Octal escapes per UTF-8 byte, so the compiler sees plain bytes.
            out.extend('\\%03o' % byte for byte in ch.encode('utf-8'))
    return '"%s"' % ''.join(out)


def existing_ssid(path):
    """The SSID in an already-generated header, or None."""
    if not path.exists():
        return None
    match = re.search(r'#define\s+SERVO_WIFI_SSID\s+"(.*)"', path.read_text())
    return match.group(1) if match and match.group(1) else None


header = pathlib.Path(env.subst('$PROJECT_DIR')) / 'include' / 'wifi_credentials.h'  # noqa: F821

# None means the variable is absent; '' means it was deliberately set to empty.
ssid = os.environ.get('WIFI_CLAITEC_SSID')
password = os.environ.get('WIFI_CLAITEC_PASS')

# An ordinary build with the variables unset must not silently destroy working
# credentials - the password cannot be recovered from anywhere else, since the
# header is the only copy on disk and it is gitignored. Keep what is there and
# say so. Setting WIFI_CLAITEC_SSID='' turns WiFi off explicitly.
kept = existing_ssid(header)
if ssid is None and password is None and kept is not None:
    print('wifi: WIFI_CLAITEC_SSID unset, keeping existing credentials for "%s"'
          % kept)
    print('wifi:   export it (or set it empty) to change this')
else:
    contents = '''/* Generated at build time from the WIFI_CLAITEC_SSID and WIFI_CLAITEC_PASS
 * environment variables by scripts/wifi_credentials.py.
 *
 * Not tracked by git, and rewritten whenever those variables are set: do not
 * edit, and do not treat this as the only record of the password.
 */
#pragma once

#define SERVO_WIFI_SSID     %s
#define SERVO_WIFI_PASSWORD %s
''' % (c_string_literal(ssid or ''), c_string_literal(password or ''))

    # Only touch the file when it changes, so unrelated builds stay cached.
    if not header.exists() or header.read_text() != contents:
        header.write_text(contents)

    if ssid:
        print('wifi: building for SSID "%s" (from WIFI_CLAITEC_SSID)' % ssid)
    elif kept is not None:
        print('wifi: WIFI_CLAITEC_SSID is empty, clearing credentials for "%s"'
              % kept)
    else:
        print('wifi: WIFI_CLAITEC_SSID is unset, WiFi will stay off')
