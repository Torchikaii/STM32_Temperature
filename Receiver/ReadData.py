import tkinter as tk
import serial
import threading

# Configure the serial connection
port = 'COM3'  # Change this to your port
baudrate = 115200  # Baud rate
timeout = 1  # Timeout in seconds

# Create a serial connection
ser = serial.Serial(port, baudrate, timeout=timeout)

class SerialReader(threading.Thread):
    def __init__(self, text_widget):
        super().__init__()
        self.text_widget = text_widget
        self.running = True

    def run(self):
        while self.running:
            if ser.in_waiting > 0:  # Check if there is data waiting in the buffer
                data = ser.readline()  # Read a line of data
                self.text_widget.insert(tk.END, data.decode('utf-8').strip() + '\n')  # Insert data into the text widget
                self.text_widget.see(tk.END)  # Scroll to the end

    def stop(self):
        self.running = False

def on_closing():
    reader.stop()  # Stop the serial reader thread
    ser.close()  # Close the serial connection
    root.quit()  # Use quit instead of destroy to exit the main loop

# Create the main window
root = tk.Tk()
root.title("Serial Data Reader")
root.geometry("400x300")
root.configure(bg='lightblue')

# Create a text area to display the data
text_area = tk.Text(root, bg='blue', fg='white', font=('Courier', 10))
text_area.pack(expand=True, fill='both')

# Start the serial reader thread
reader = SerialReader(text_area)
reader.start()

# Set the closing protocol
root.protocol("WM_DELETE_WINDOW", on_closing)

# Start the Tkinter main loop
root.mainloop()
