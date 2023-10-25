import NimbleView from "./NimbleView";

class DARTRemote {
  url: string;
  view: NimbleView;
  socket: WebSocket | null;

  constructor(url: string, view: NimbleView) {
    this.url = url;
    this.view = view;

    this.trySocket();

    window.addEventListener("keydown", (e: KeyboardEvent) => {
      const message = JSON.stringify({
        type: "keydown",
        key: e.key.toString(),
      });
      if (this.socket != null && this.socket.readyState == WebSocket.OPEN) {
        this.socket.send(message);
      }
    });

    window.addEventListener("keyup", (e: KeyboardEvent) => {
      const message = JSON.stringify({
        type: "keyup",
        key: e.key.toString(),
      });
      if (this.socket != null && this.socket.readyState == WebSocket.OPEN) {
        this.socket.send(message);
      }
    });

    this.view.addDragListener((key: string, pos: number[]) => {
      const message = JSON.stringify({
        type: "drag",
        key,
        pos,
      });
      if (this.socket != null && this.socket.readyState == WebSocket.OPEN) {
        this.socket.send(message);
      }
    });
  }

  /**
   * This reads and handles a command sent from the backend
   */
  handleCommand = (command: Command) => {
    // We manually handle any "interactive" commands here, since the NimbleView object by itself doesn't know what to do with those.
    if (command.type === "create_button") {
      this.view.createButton(
        command.key,
        command.from_top_left,
        command.size,
        command.label,
        () => {
          const message = JSON.stringify({
            type: "button_click",
            key: command.key,
          });
          if (this.socket != null && this.socket.readyState == WebSocket.OPEN) {
            this.socket.send(message);
          }
        }
      );
    } else if (command.type === "create_slider") {
      this.view.createSlider(
        command.key,
        command.from_top_left,
        command.size,
        command.min,
        command.max,
        command.value,
        command.only_ints,
        command.horizontal,
        (new_value: number) => {
          const message = JSON.stringify({
            type: "slider_set_value",
            key: command.key,
            value: new_value,
          });
          if (this.socket != null && this.socket.readyState == WebSocket.OPEN) {
            this.socket.send(message);
          }
        }
      );
    } else {
      // Otherwise, the command doesn't require any interactive callbacks, so NimbleView can handle it directly.
      this.view.handleCommand(command);
    }
  };

  /**
   * This attempts to connect a socket to the backend.
   */
  trySocket = () => {
    this.socket = new WebSocket(this.url);

    // Connection opened
    this.socket.addEventListener("open", (event) => {
      console.log("Socket connected!");
      // Clear the view on a reconnect, the socket will broadcast us new data
      this.view.setConnected(true);
      this.view.clear();
    });

    // Listen for messages
    this.socket.addEventListener("message", (event) => {
      try {
        const data: Command[] = JSON.parse(event.data);
        data.forEach(this.handleCommand);
        this.view.render();
      } catch (e) {
        console.error(
          "Something went wrong on command:\n\n" + event.data + "\n\n",
          e
        );
      }
    });

    this.socket.addEventListener("close", () => {
      // do nothing
      console.log("Socket closed. Retrying in 1s");
      this.view.stop();
      this.view.setConnected(false);
      setTimeout(this.trySocket, 1000);
    });
  };
}

export default DARTRemote;
