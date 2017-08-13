from communicator import Communicator
from motor_controller import MotorController
import tkinter as tk
import consts

def packet_to_dict(pkt, fmt):
    return {key: pkt[ind] for key, ind in fmt.items()}

def listener(data):
    print("received", data)
    # if len(data) >= 1:
        # header = data[0]
        # body = data[1:]
        # if (header == HDR_GETSTATUS):
            # try:
                # unpacked = packet_to_dict(unpack("<Hhhhhhhh", body), FMT_GETSTATUS)                 
                # for key, val in unpacked.items():
                    # coreStatusLabels[key].set(val)
                # if boolAutoPoll.get():
                    # refresh()
            # except Exception as e:
                # print(e)

def on_closing():
    c.close()
    win.destroy()

def send_speed():
	mc.set_motors(float(str_left.get()), float(str_right.get()))

if __name__ == "__main__":	
	comPort = input("Which COM port to use: ")
		
	win = tk.Tk()
	win.title("Robot Controller")

	frame = tk.Frame(win)
	frame.pack(fill = tk.BOTH, expand = 1)
		
	str_left = tk.StringVar()
	str_left.set("0")
	
	str_right = tk.StringVar()
	str_right.set("0")
				 
	c = Communicator(115200, comPort, listener) 
	c.open()
	
	mc = MotorController(c)

	tk.Entry(frame, textvariable = str_left).grid(column = 0, row = 0, sticky="WE")
	tk.Entry(frame, textvariable = str_right).grid(column = 1, row = 0, sticky="WE")
	tk.Button(frame, text = "Send Speeds", command = send_speed).grid(column = 0, row = 1, sticky="WE")

	tk.Grid.rowconfigure(frame, 0, weight=1)

	tk.Grid.columnconfigure(frame, 0, weight=1)
	tk.Grid.columnconfigure(frame, 1, weight=1)

	win.protocol("WM_DELETE_WINDOW", on_closing)

	win.mainloop()