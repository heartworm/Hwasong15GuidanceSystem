import communicator as com
import tkinter as tk
from consts import * 
from struct import *

def packet_to_dict(pkt, fmt):
    return {key: pkt[ind] for key, ind in fmt.items()}

def listener(data):
    return
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

def send_pwm():
    newPwm = int(strPwm.get())
    print("new motor val ", newPwm)
    bytes_out = pack("<BBB", 0x01, 0x00, newPwm)
    print("Bytes sent: ", bytes_out)
    c.write(bytes_out)
    
if __name__ == "__main__":	
	comPort = input("Which COM port to use: ")
		
	win = tk.Tk()
	win.title("Robot Controller")

	frame = tk.Frame(win)
	frame.pack(fill = tk.BOTH, expand = 1)
		
	strPwm = tk.StringVar()
	strPwm.set("100")
				 
	c = com.Communicator(115200, comPort, listener) 
	c.open()

	tk.Entry(frame, textvariable = strPwm).grid(column = 0, row = 0, sticky="WE")
	tk.Button(frame, text = "Send PWM", command = send_pwm).grid(column = 1, row = 0, sticky="WE")

	tk.Grid.rowconfigure(frame, 0, weight=1)

	tk.Grid.columnconfigure(frame, 0, weight=1)

	win.protocol("WM_DELETE_WINDOW", on_closing)

	win.mainloop()