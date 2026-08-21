# PyServer.py

`PyServer.py` is a small TCP socket server that lets the CODESYS side of ArfBotOS trigger Python vision scripts on another machine, typically a Raspberry Pi or vision host. It listens for plain-text commands, runs the requested script inside the same Python process, captures the script's printed output, and sends that output back to the client.

## What The Server Does

At a high level the server works like this:

1. Start a TCP listener on port `50011`.
2. Accept incoming client connections on any network interface.
3. Spawn a connection thread for each client.
4. For each received command:
   - acknowledge the request by sending back a thread ID string
   - treat the received text as a command line
   - extract the script path and arguments
   - execute the target Python script with `runpy.run_path()`
   - capture anything the script prints to stdout
   - send the captured text back to the client prefixed with `res: `

This makes the server a thin transport layer around the existing vision scripts rather than a separate vision engine by itself.

## Listener Setup

`main()` creates a TCP socket with:

- `AF_INET`
- `SOCK_STREAM`
- `SO_REUSEADDR`

It binds to:

- host: `''`
- port: `50011`

The blank host means it listens on all available interfaces, which matches the comment in the code about hosting vision on a separate machine.

## Connection Model

Each accepted socket connection gets its own `handle_connection()` thread. That thread:

- immediately sends a message like `connection uid: <thread_id>`
- polls the socket in non-blocking mode
- creates a new command thread for each normal command it receives

The receive call uses:

```python
connection.recv(4096, 0x40)
```

where `0x40` is `MSG_DONTWAIT`, so the loop keeps running without blocking on input.

## Command Format

The server expects a plain-text command string. In practice it is designed around commands like:

```text
sudo python /var/opt/codesys/PlcLogic/Application/Vision/FastTemplateMatching.py -s /var/opt/codesys/PlcLogic/Application/Vision/outputimage.bmp -t /var/opt/codesys/PlcLogic/Application/Vision/Templates/battery.jpg -i 5 -j 0.0 -k 0.8 -l 90.0 -d True
```

That is the same style used in [`TestClient.py`](c:/Users/dalet/Github/ArfBotOS/OpenCV/SocketServer/TestClient.py).

Inside `handle_command()` the server:

1. Decodes the bytes into text.
2. Removes non-ASCII characters with `remove_non_ascii()`.
3. Splits the command string on spaces.
4. Assumes element `2` is the Python script path.
5. Sets `sys.argv` to `list[2:]`.
6. Runs that script as `__main__` with `runpy.run_path()`.

Because of that parsing, the incoming command is expected to look like:

- token 0: launcher such as `sudo`
- token 1: interpreter such as `python`
- token 2: script path
- remaining tokens: arguments for that script

## Response Format

The server writes several status messages back to the client over the same socket.

Typical sequence:

1. `connection uid: <id>`
2. `command uid: <id>`
3. `res: <stdout from the script>`

The final response is whatever the called script printed. For example:

- `FastTemplateMatching.py` prints `LOC ...` lines
- `CalibrateCamera.py` prints a `CAL ...` line

`PyServer.py` does not parse those payloads. It simply forwards them as text after adding the `res: ` prefix.

## Kill Command

`handle_connection()` also recognizes messages beginning with:

```text
kill: <thread_id>
```

When it receives that pattern, it pushes the thread ID into a shared queue and responds with:

```text
res: killing thread with id: <thread_id>
```

There is a partially implemented worker-thread path intended to monitor that queue and stop running work, but the active code path currently launches `handle_command()` threads directly. That means kill support appears to be unfinished in the current implementation.

## Execution Strategy

The active implementation uses:

- `runpy.run_path()`
- `redirect_stdout()`

This means the target script runs inside the server's Python process and any `print()` output is captured in memory and sent back to the client.

There is also a commented-out `subprocess.run()` path. The comment notes that subprocess execution is slower because the Python modules must be loaded again for every command.

## Threading And Locks

The server uses several global locks:

- `print_lock` to serialize console logging
- `connection_lock` to serialize socket sends
- `queue_lock` for the thread-kill queue helper

This helps avoid interleaved console output and overlapping writes when several threads use the same connection.

## Related Files

- [`TestClient.py`](c:/Users/dalet/Github/ArfBotOS/OpenCV/SocketServer/TestClient.py) is a simple local test client that opens a socket, sends a vision command, and prints whatever the server returns.
- [`VisionErrorLog.txt`](c:/Users/dalet/Github/ArfBotOS/OpenCV/SocketServer/VisionErrorLog.txt) contains example runtime errors seen while invoking the vision scripts, such as missing `cal.yaml` or `roi.yaml`.

## Current Notes

- The server is built around plain-text commands, not a structured JSON protocol.
- `receive_list()` exists for a length-prefixed JSON format, but it is not used in the active path.
- `is_socket_closed()` is defined but not used in the current server loop.
- Command parsing is simple `split(" ")`, so it will not handle quoted paths or arguments with spaces robustly.
- Because `runpy` executes scripts in-process, those scripts share the server process state, including `sys.argv`.
- Error handling is minimal: many exceptions are caught and ignored, and execution errors are mostly printed server-side rather than returned as structured error responses.
