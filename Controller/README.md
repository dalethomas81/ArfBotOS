# Controller

The `Controller/` folder contains the Python bridge that connects a PlayStation 5 DualSense controller to the rest of ArfBotOS. The main piece is a TCP server that talks to the physical controller through `pydualsense` and exposes that controller over a simple socket protocol so the CODESYS side of the project can read controller state and send feedback commands.

## Files

- [`DualSenseServer.py`](c:/Users/dalet/Github/ArfBotOS/Controller/DualSenseServer.py): the actual controller bridge server
- [`DualSenseClient.py`](c:/Users/dalet/Github/ArfBotOS/Controller/DualSenseClient.py): a small local test client for debugging the server

## High-Level Design

The controller subsystem works like this:

1. `DualSenseServer.py` opens a TCP server on port `50013`.
2. A client connects to that server.
3. The server initializes a `pydualsense` device connection.
4. On a cycle, the server sends the current controller state bytes to the client.
5. The client can also send plain-text commands back to the server to control controller outputs such as:
   - light color
   - player LEDs
   - microphone mute state
   - rumble motors
   - adaptive trigger mode and force

This gives ArfBotOS two-way communication with the controller:

- controller inputs flow from the DualSense into the TCP socket
- haptics and LEDs flow from the TCP socket back to the DualSense

## Network Setup

`DualSenseServer.py` uses:

- host: `localhost`
- port: `50013`

The CODESYS project references the same port in `Codesys/ArfBot.xml`, so the intended deployment is that the PLC/runtime side connects to this local Python server and exchanges controller data over that socket.

## How `DualSenseServer.py` Works

### Startup

`main()`:

- creates a TCP socket
- enables `SO_REUSEADDR`
- binds to `localhost:50013`
- listens for connections
- spawns a new connection thread for each accepted client

The server also installs `SIGINT` and `SIGTERM` handlers that set global shutdown flags.

### Connection lifecycle

Each client is handled by `handle_connection(connection)`.

That function:

1. Monitors the socket to see whether the client is still connected.
2. Creates and initializes a `pydualsense()` device.
3. Gives a short rumble sequence when the controller is connected.
4. Enters a loop that:
   - checks whether the controller is still connected
   - checks for incoming client commands
   - sends controller state updates on a timed cycle
5. Gives another short rumble sequence on disconnect.
6. Closes the DualSense device handle.

If the controller is not available, the code catches the exception and keeps retrying in the outer loop.

## State Streaming

While connected, the server sends controller data about every `0.05` seconds.

The payload format is:

```text
states: <raw controller state bytes>
```

In the code this is assembled by:

1. encoding the ASCII prefix `states: `
2. appending `dualsense.states`
3. sending the combined byte array over the socket

This means the protocol is not plain JSON or plain text. It is a short text prefix followed by the raw byte state from `pydualsense`.

The CODESYS side of the project looks for the same `states: ` prefix, which is how the PLC knows it is receiving controller-state data.

## Command Handling

The client can send text commands in this format:

```text
cmd: <CommandName>:<parameters>
```

`handle_client_message()` parses the command name and routes it to a handler function.

Supported commands currently include:

- `cmd: setColorI:r,g,b`
- `cmd: setMicrophoneState:true_or_false`
- `cmd: setPlayerID:value`
- `cmd: setBrightness:value`
- `cmd: setRightMotor:intensity,duration`
- `cmd: setLeftMotor:intensity,duration`
- `cmd: setRightTriggerMode:value`
- `cmd: setLeftTriggerMode:value`
- `cmd: setRightTriggerForce:index,force`
- `cmd: setLeftTriggerForce:index,force`

After a command is processed, the server sends back:

```text
res:
```

That response is only an acknowledgement. It does not include a structured result payload.

## Output Features Exposed By The Server

### Light bar color

`setColorI:r,g,b`

Sets the controller light bar using integer RGB values from `0` to `255`.

### Microphone LED / mute state

`setMicrophoneState:true_or_false`

Passes a boolean into `dualsense.audio.setMicrophoneState(...)`.

### Player LEDs

`setPlayerID:value`

Maps the numeric value to a `PlayerID` constant:

- `4` = player 1
- `10` = player 2
- `21` = player 3
- `27` = player 4
- `31` = all

### Brightness

`setBrightness:value`

Maps:

- `0` = high
- `1` = medium
- `2` = low

### Rumble motors

`setLeftMotor:intensity,duration`

`setRightMotor:intensity,duration`

These start the selected motor immediately and launch a timer thread that turns that motor back off after the requested duration.

### Adaptive trigger mode

`setLeftTriggerMode:value`

`setRightTriggerMode:value`

The code maps numeric values `0` through `9` onto the `TriggerModes` enum exposed by `pydualsense`, including:

- off
- rigid
- pulse
- combined rigid and pulse modes
- calibration

### Adaptive trigger force

`setLeftTriggerForce:index,force`

`setRightTriggerForce:index,force`

These pass the two integers directly to `dualsense.triggerL.setForce(...)` or `dualsense.triggerR.setForce(...)`.

## Threading And Shutdown

The server uses:

- a `print_lock` to keep console logs readable
- a `connection_lock` to serialize socket writes

It also uses two global flags:

- `disconnect_controller`
- `shutdown`

These are set during cleanup and are also used by the rumble stop threads and connection loops.

## `DualSenseClient.py`

`DualSenseClient.py` is a very small debugging client.

It:

1. connects to `localhost:50013`
2. every 5 seconds sends:

```text
cmd: setColorI:0,0,255
```

3. continuously prints received bytes in hex

This is useful for verifying that:

- the server accepts commands
- the controller responds
- `states:` traffic is being produced

## How It Connects To ArfBotOS

- The CODESYS project connects to this server on port `50013`.
- The PLC side looks for the `states: ` prefix in incoming traffic.
- The PLC side also builds outgoing commands such as `cmd: setColorI:...` and `cmd: setRightMotor:...`.

This makes the Python server the hardware-facing controller adapter, while the PLC remains the main application-side consumer and command source.

## Current Notes

- The server binds to `localhost`, so it is intended for same-machine communication unless the host is changed.
- The socket reads use non-blocking mode with `0x40` (`MSG_DONTWAIT`).
- Error handling is intentionally loose in several places; many exceptions are caught and ignored so the server keeps retrying.
- The command-line option parsing at the bottom of `DualSenseServer.py` is placeholder scaffolding and does not affect the main controller behavior.
- The disconnect and shutdown flow works through globals rather than a more structured lifecycle manager.
