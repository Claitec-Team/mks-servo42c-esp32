"""Generate include/wifi_credentials.h from the environment.

Keeps WiFi credentials out of the repository: the header is gitignored and is
rewritten on every build from WIFI_CLAITEC_SSID and WIFI_CLAITEC_PASS.

Writing a header rather than passing -D flags avoids shell quoting entirely, so
credentials containing spaces, quotes or non-ASCII characters work unchanged.
"""

import os
import pathlib

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


ssid = os.environ.get('WIFI_CLAITEC_SSID', '')
password = os.environ.get('WIFI_CLAITEC_PASS', '')

contents = '''/* Generated at build time from the WIFI_CLAITEC_SSID and WIFI_CLAITEC_PASS
 * environment variables by scripts/wifi_credentials.py.
 *
 * Not tracked by git, and overwritten on every build: do not edit.
 */
#pragma once

#define SERVO_WIFI_SSID     %s
#define SERVO_WIFI_PASSWORD %s
''' % (c_string_literal(ssid), c_string_literal(password))

header = pathlib.Path(env.subst('$PROJECT_DIR')) / 'include' / 'wifi_credentials.h'  # noqa: F821

# Only touch the file when it changes, so unrelated builds are not invalidated.
if not header.exists() or header.read_text() != contents:
    header.write_text(contents)

if ssid:
    print('wifi: building for SSID "%s" (from WIFI_CLAITEC_SSID)' % ssid)
else:
    print('wifi: WIFI_CLAITEC_SSID is unset, WiFi will stay off')
