# Archived PC tools

These scripts target deprecated GPS/TCP experiments and are retained only for
history. They are not part of the current UDP telemetry/control workflow and
are not included in current Python verification.

`test_tcp_legacy.py` was intended to check drone-to-PC TCP packet latency and
integrity while one endpoint acted as the listening server. It predates the
current UDP 4210 workflow and is not a supported protocol test.
