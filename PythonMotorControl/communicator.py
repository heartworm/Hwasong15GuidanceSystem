import serial as s
import threading

FLG_ESC = 0xFF
FLG_ETX = 0xFE
FLG_STX = 0xFD

class Communicator:
	def __init__(self, baud, port, listener):
		self.recv_buffer = bytearray()
		self.in_frame = False
		self.in_esc = False
		
		self.write_buffer = bytearray()
		
		self.read_thread = None
		self.write_thread = None
		self.die = False
	
		self.ser = s.Serial()
		self.ser.baudrate = baud
		self.ser.port = port
		self.ser.timeout = 0
		
		self.listener = listener
		
		
	def is_open(self):
		return self.ser.is_open
	
	def __enter__(self):
		self.open()
		return self
	
	def open(self):
		self.ser.open()
		self.start_reading()
		self.start_writing()
		
	def __exit__(self):
		self.close()
	
	def close(self):
		self.die = True
		if self.read_thread is not None and self.read_thread.is_alive():
			self.read_thread.join()
		if self.write_thread is not None and self.write_thread.is_alive():
			self.write_thread.join()	
		self.ser.close()
		
	def start_reading(self):
		self.read_thread = threading.Thread(target = self.read_task)
		self.read_thread.start()
			
	def start_writing(self):
		self.write_thread = threading.Thread(target = self.write_task)
		self.write_thread.start()
		
	def write(self, data):
		self.write_buffer.append(FLG_STX)
		for byte_out in data:
			if byte_out in [FLG_STX, FLG_ETX, FLG_ESC]:
				self.write_buffer.append(FLG_ESC)
			self.write_buffer.append(byte_out)
		self.write_buffer.append(FLG_ETX)	
		
	def write_task(self):
		while not self.die:
			out_len = len(self.write_buffer)
			if out_len > 0:
				bytes_out = self.write_buffer[0:out_len]
				self.ser.write(bytes_out)
				del self.write_buffer[0:out_len]
	
	def read_task(self):
		while not self.die:
			bytes_in = self.ser.read()
			if bytes_in is not None and len(bytes_in) > 0:
				byte_in = bytes_in[0]
				if not self.in_esc:
					if byte_in == FLG_STX:
						self.clear_buffer()	
						self.in_frame = True
					elif byte_in == FLG_ETX:
						if self.in_frame: self.listener(self.recv_buffer)
						self.clear_buffer()
					elif byte_in == FLG_ESC and self.in_frame:
						self.in_esc = True
					elif self.in_frame:
						self.recv_buffer.append(byte_in)				
				else:
					self.in_esc = False
					self.recv_buffer.append(byte_in)
		
	def clear_buffer(self):
		del self.recv_buffer[:]
		self.in_frame = False
		self.in_esc = False
	