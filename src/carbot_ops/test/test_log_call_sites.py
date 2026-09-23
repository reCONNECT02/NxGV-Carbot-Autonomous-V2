"""rclpy raises 'Logger severity cannot be changed between calls' when ONE call site logs at
two severities, e.g. (log.info if ok else log.warn)(msg). Seen on risabot5 in
calibration_wizard._srv: after one refused action every later action answered
'Internal error'. Keep such call sites out of every package."""
import os
import re

SRC = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..'))
PATTERN = re.compile(r'\(\s*[\w.()]*get_logger\(\)\.\w+\s+if\b')


def test_no_mixed_severity_call_sites():
    bad = []
    for pkg in os.listdir(SRC):
        for root, _dirs, files in os.walk(os.path.join(SRC, pkg)):
            if os.sep + 'test' in root or 'YDLidar' in root:
                continue
            for f in files:
                if f.endswith('.py'):
                    path = os.path.join(root, f)
                    with open(path, encoding='utf-8', errors='replace') as fh:
                        for n, line in enumerate(fh, 1):
                            if PATTERN.search(line):
                                bad.append(f'{os.path.relpath(path, SRC)}:{n}: {line.strip()}')
    assert not bad, 'one call site, two severities (rclpy raises):\n' + '\n'.join(bad)
