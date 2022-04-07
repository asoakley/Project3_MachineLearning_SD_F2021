import socket
import threading
import tkinter

HOST = '192.168.1.35'
PORT = 9099

class Client:

    def __init__(self, host, port):

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.connect((host,port))

        self.gui_done = False
        self.running = True

        gui_thread = threading.Thread(target=self.gui_loop)
        receive_thread = threading.Thread(target=self.receive)

        gui_thread.start()
        receive_thread.start()

    def gui_loop(self):
        self.win = tkinter.Tk()
        self.win.configure(bg="red")
        self.label = tkinter.Label(self.win, text="Hello World!", bg="lightgray")
        self.label.config(font=("Calibri", 12))
        self.label.pack(padx=20, pady=5)

        self.high_button = tkinter.Button(self.win, text="High", command=self.on)
        self.high_button.config(font=("Calibri", 12))
        self.high_button.pack(padx=20, pady=5)

        self.low_button = tkinter.Button(self.win, text="Low", command=self.off)
        self.low_button.config(font=("Calibri", 12))
        self.low_button.pack(padx=20, pady=5)

        self.textframe = tkinter.Frame(self.win, width=500, height=100, bg="lightgray")
        self.textframe.pack(padx=5, pady=5)

        self.confidence = tkinter.Text(self.textframe, width=40, height=10)
        self.confidence.pack(padx=5, pady=5)
        self.confidence.insert('1.0', 'test\r\n')

        self.gui_done = True

        self.win.protocol("WM_DELETE_WINDOW", self.stop)

        self.win.mainloop()


    def receive(self):
        while self.running:
            try:
                message = self.sock.recv(1024)
                self.confidence.insert('end', message)
            except ConnectionAbortedError:
                break
            except:
                print("Error")
                self.sock.close()
                break


    def stop(self):
        self.running = False
        self.win.destroy()
        self.sock.close()
        exit(0)

    def on(self):
        message = f"GET /H\n"
        self.sock.send(message.encode('utf-8'))

    def off(self):
        message = f"GET /L\n"
        self.sock.send(message.encode('utf-8'))

client = Client(HOST,PORT)
