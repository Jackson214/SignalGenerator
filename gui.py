# Jackson Spray
# GUI Running on OS connected to Microcontroller via USB

from tkinter import *
from tkinter import ttk
import tkinter.scrolledtext as st
import serial
import io
import time

import numpy as np
import matplotlib.pyplot as plt
from matplotlib import ticker as mticker


GainA = -376.6
Ra = 0x7EA

points = [None] * 1024
ser = serial.Serial('COM3', 115200, bytesize=serial.EIGHTBITS, timeout=2, parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE, rtscts=0)
sio = io.TextIOWrapper(io.BufferedRWPair(ser, ser))

root = Tk()
canvas = Canvas(root, width=1024, height=300, bg="gray90")
voltage_var = StringVar()
freq_var = StringVar()
offset_var = StringVar()
dutyCycle_var = StringVar()
phaseShift_var = StringVar()
channel_var = StringVar()
cycles_var = StringVar()
fstart_var = StringVar()
fstop_var = StringVar()

differential = False
showVoltages = False
hilbert = False
level = False

freqList = []
dbList = []


def lineToPoints(x0, y0, x1, y1):
        #"Bresenham's line algorithm"
        points_in_line = []
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        x, y = x0, y0
        sx = -1 if x0 > x1 else 1
        sy = -1 if y0 > y1 else 1
        if dx > dy:
            err = dx / 2.0
            while x != x1:
                points_in_line.append((x, y))
                err -= dy
                if err < 0:
                    y += sy
                    err += dx
                x += sx
        else:
            err = dy / 2.0
            while y != y1:
                points_in_line.append((x, y))
                err -= dx
                if err < 0:
                    x += sx
                    err += dy
                y += sy
        points_in_line.append((x, y))
        return points_in_line


def savePosn(event):
    global lastx, lasty
    lastx, lasty = event.x, event.y
    

def addLine(event):
    canvas.create_line((lastx, lasty, event.x, event.y))
    a = lineToPoints(lastx,lasty,event.x,event.y)
    global points
    for point in a:
    	#put into point array
    	try:
    		points[point[0]] = point[1]
    	except:
			continue
	
	savePosn(event)

def startCmd():
	sio.write("start\n")
	sio.flush()

def stopCmd():
	sio.write("stop\n")
	sio.flush()

def resetCmd():
	sio.write("reset\n")
	sio.flush()
	time.sleep(1)
	buff = sio.readlines()
	for line in buff:
		print(str(line))
		uartAppend(outputText, str(line))

def dcCmd():
	volts = voltage_var.get()
	channel = channel_var.get()
	sio.write(f"dc {channel} {volts}\n")
	sio.flush()

def sineCmd():
	volts = voltage_var.get()
	channel = channel_var.get()
	phaseShift = phaseShift_var.get()
	freq = freq_var.get()
	offset = offset_var.get()
	sio.write(f"sine {channel} {freq} {volts} {offset} {phaseShift}\n")
	sio.flush()

def squareCmd():
	volts = voltage_var.get()
	channel = channel_var.get()
	freq = freq_var.get()
	offset = offset_var.get()
	dutyCycle = dutyCycle_var.get()
	sio.write(f"square {channel} {freq} {volts} {offset} {dutyCycle}\n")
	sio.flush()

def sawtoothCmd():
	volts = voltage_var.get()
	channel = channel_var.get()
	freq = freq_var.get()
	offset = offset_var.get()
	sio.write(f"sawtooth {channel} {freq} {volts} {offset}\n")
	sio.flush()

def triangleCmd():
	volts = voltage_var.get()
	channel = channel_var.get()
	freq = freq_var.get()
	offset = offset_var.get()
	sio.write(f"triangle {channel} {freq} {volts} {offset}\n")
	sio.flush()

def cyclesCmd():
	channel = channel_var.get()
	num = cycles_var.get()
	sio.write(f"cycles {channel} {num}\n")
	sio.flush()

def differentialCmd():
	global differential
	differential = not differential
	if (differential):
		tx = "1"
	else:
		tx = "0"
	sio.write(f"differential {tx}\n")
	sio.flush()

def hilbertCmd():
	global hilbert
	hilbert = not hilbert
	if (hilbert):
		tx = "1"
	else:
		tx = "0"
	sio.write(f"hilbert {tx}\n")
	sio.flush()

def voltDisplayCmd():
	channel = channel_var.get()
	global showVoltages
	showVoltages = not showVoltages
	sio.write(f"voltage {channel}\n")
	sio.flush()
	buff = sio.readlines()
	for line in buff:
		print(str(line))
		uartAppend(outputText, str(line))
	showVoltages = not showVoltages
	sio.write(f"voltage {channel}\n")
	sio.flush()

def bufferGetCmd():
	buff = sio.readlines()
	for line in buff:
		print(str(line))
		uartAppend(outputText, str(line))

def levelCmd():
	global level
	level = not level
	if (level):
		tx = "on"
	else:
		tx = "off"
	sio.write(f"level {tx}\n")
	sio.flush()
	time.sleep(1)
	buff = sio.readlines()
	for line in buff:
		print(str(line))
		uartAppend(outputText, str(line))

def gainCmd():
	fstart = fstart_var.get()
	fstop = fstop_var.get()
	sio.write(f"gain {fstart} {fstop}\n")
	sio.flush()
	time.sleep(5)
	global freqList
	global dbList
	dbList.clear()
	freqList.clear()
	freqLine = False
	dbLine = False
	buff = sio.readlines()
	for line in buff:
		if "DB ARRAY" in line:
			dbLine = True
			freqLine = False
		if "FREQ ARRAY" in line:
			dbLine = False
			freqLine = True
		if "." in line:
			if (dbLine):
				dbList.append(float(line))
			if (freqLine):
				freqList.append(float(line))
		print(str(line))
	print("db list:")
	print(dbList)
	print("freq list:")
	print(freqList)

	xpoints = np.array(freqList)
	ypoints = np.array(dbList)

	plt.plot(xpoints, ypoints)
	plt.xscale("log")
	plt.xlabel("Frequency (Hz)")
	plt.ylabel("Gain (dB)")
	plt.title("Bode Plot Diagram")
	plt.show()
	uartAppend(outputText, "Gain Calculated")


def printData():
	#Correct Points:
	freq = freq_var.get()
	for i in range (0, len(points)-1):
		if points[i] == None:
			points[i] = 0
		if points[i] < 0:
			points[i] = 0
		if points[i] > 300:
			points[i] = 300

	#Convert Points to R Value
	for i in range (0, len(points)-1):
		points[i] = points[i] - 150

	for i in range (0, len(points)):
		points[i] = (points[i] / 150) * -1 # Makes point between 1 and -1
		points[i] = int((GainA * points[i] * 4.5) + Ra) # Should calculate correct R value
		points[i] = int(points[i] + 0x3000) # Add in channel A

	pointsStr = str(points)
	new1 = pointsStr.replace("[", "")
	new2 = new1.replace("]", "")
	pointsForm = new2.replace(",", "")

	sio.write(f"arb {freq}\n")

	sio.flush()
	time.sleep(1)

	i = 0
	for i in range(128):
		try:
			if ((7+(i*8)) == 1023):
				sio.write(pointsForm.split(' ')[0+(i*8)]+" "+pointsForm.split(' ')[1+(i*8)]+" "+pointsForm.split(' ')[2+(i*8)]+" "+pointsForm.split(' ')[3+(i*8)]+" "+pointsForm.split(' ')[4+(i*8)]+" "+pointsForm.split(' ')[5+(i*8)]+" "+pointsForm.split(' ')[6+(i*8)]+" "+pointsForm.split(' ')[6+(i*8)]+"\n")
			else:
				sio.write(pointsForm.split(' ')[0+(i*8)]+" "+pointsForm.split(' ')[1+(i*8)]+" "+pointsForm.split(' ')[2+(i*8)]+" "+pointsForm.split(' ')[3+(i*8)]+" "+pointsForm.split(' ')[4+(i*8)]+" "+pointsForm.split(' ')[5+(i*8)]+" "+pointsForm.split(' ')[6+(i*8)]+" "+pointsForm.split(' ')[7+(i*8)]+"\n")
		except:
			print(f"i = {i}")
		sio.flush()
		time.sleep(0.01)

	buff = sio.readlines()
	for line in buff:
		print(str(line))
		uartAppend(outputText, str(line))
	for point in points:
		point = 0
	print("Done")
	uartAppend(outputText, "Done Sending Arb Data")

def clearCmd():
	canvas.delete('all')
	canvas.create_line(0,150,1024,150, fill="gray", width=2)
	for point in points:
		point = 0

def uartAppend(outputText, text):
	newText = text.replace("\n", "")
	outputText.config(state ='normal')
	outputText.insert(END,newText+"\n")
	outputText.configure(state='disabled')
	outputText.see("end")

canvas.pack(pady=20)
canvas.create_line(0,150,1024,150, fill="gray", width=2)
canvas.bind("<Button-1>", savePosn)
canvas.bind("<B1-Motion>", addLine)

frame = Frame(root)
frame.pack()

startButton = ttk.Button(frame, text='RUN', command=startCmd)
startButton.grid(row=2,column=6,padx=50,pady=1)
stopButton = ttk.Button(frame, text='STOP', command=stopCmd)
stopButton.grid(row=3,column=6,padx=5,pady=1)
resetButton = ttk.Button(frame, text='RESET', command=resetCmd)
resetButton.grid(row=4,column=6,padx=5,pady=1)

channelText = Label(frame, text = "Channel:").grid(row=0,column=0)
channelBox = ttk.Entry(frame,textvariable = channel_var, font=('calibre',10,'normal'))
channelBox.grid(row=0,column=1,padx=5,pady=2)

voltageText = Label(frame, text = "Voltage/Amplitude:").grid(row=1,column=0)
voltageBox = ttk.Entry(frame,textvariable = voltage_var, font=('calibre',10,'normal'))
voltageBox.grid(row=1,column=1,padx=5,pady=2)

freqText = Label(frame, text = "Frequency:").grid(row=2,column=0)
freqBox = ttk.Entry(frame,textvariable = freq_var, font=('calibre',10,'normal'))
freqBox.grid(row=2,column=1,padx=5,pady=2)

offsetText = Label(frame, text = "Offset:").grid(row=3,column=0)
offsetBox = ttk.Entry(frame,textvariable = offset_var, font=('calibre',10,'normal'))
offsetBox.grid(row=3,column=1,padx=5,pady=2)

dutyCycleText = Label(frame, text = "Duty Cycle:").grid(row=4,column=0)
dutyCycleBox = ttk.Entry(frame,textvariable = dutyCycle_var, font=('calibre',10,'normal'))
dutyCycleBox.grid(row=4,column=1,padx=5,pady=2)

phaseShiftText = Label(frame, text = "Phase Shift:").grid(row=5,column=0)
phaseShiftBox = ttk.Entry(frame,textvariable = phaseShift_var, font=('calibre',10,'normal'))
phaseShiftBox.grid(row=5,column=1,padx=5,pady=2)

dcButton = ttk.Button(frame, text='DC', command=dcCmd)
dcButton.grid(row=0,column=2,padx=5,pady=1)

sineButton = ttk.Button(frame, text='Sine', command=sineCmd)
sineButton.grid(row=1,column=2,padx=5,pady=1)

squareButton = ttk.Button(frame, text='Square', command=squareCmd)
squareButton.grid(row=2,column=2,padx=5,pady=1)

sawtoothButton = ttk.Button(frame, text='Sawtooth', command=sawtoothCmd)
sawtoothButton.grid(row=3,column=2,padx=5,pady=1)

triangleButton = ttk.Button(frame, text='Triangle', command=triangleCmd)
triangleButton.grid(row=4,column=2,padx=5,pady=1)

arbButton = ttk.Button(frame, text='Arbitrary', command=printData)
arbButton.grid(row=5,column=2,padx=5,pady=1)

NcyclesText = Label(frame, text = "# Cycles:").grid(row=0,column=3)
NcyclesBox = ttk.Entry(frame,textvariable = cycles_var, font=('calibre',10,'normal'))
NcyclesBox.grid(row=1,column=3,padx=5,pady=2)

FstartText = Label(frame, text = "Start Frequency:").grid(row=2,column=3)
FstartBox = ttk.Entry(frame,textvariable = fstart_var, font=('calibre',10,'normal'))
FstartBox.grid(row=3,column=3,padx=5,pady=2)

FstopText = Label(frame, text = "Stop Frequency:").grid(row=4,column=3)
FstopBox = ttk.Entry(frame,textvariable = fstop_var, font=('calibre',10,'normal'))
FstopBox.grid(row=5,column=3,padx=5,pady=2)

toggleText = Label(frame, text = "Toggle Buttons:").grid(row=0,column=5)

cyclesButton = ttk.Button(frame, text='Cycles', command=cyclesCmd)
cyclesButton.grid(row=1,column=4,padx=5,pady=1)

differentialButton = ttk.Button(frame, text='Differential', command=differentialCmd)
differentialButton.grid(row=1,column=5,padx=5,pady=1)

hilbertButton = ttk.Button(frame, text='Hilbert', command=hilbertCmd)
hilbertButton.grid(row=2,column=5,padx=5,pady=1)

voltageDisplayButton = ttk.Button(frame, text='Voltage IN', command=voltDisplayCmd)
voltageDisplayButton.grid(row=3,column=5,padx=5,pady=1)

# bufferGetButton = ttk.Button(frame, text='Get Buffer', command=bufferGetCmd)
# bufferGetButton.grid(row=4,column=5,padx=5,pady=1)

levelButton = ttk.Button(frame, text='Level', command=levelCmd)
levelButton.grid(row=4,column=5,padx=5,pady=1)

gainButton = ttk.Button(frame, text='Gain', command=gainCmd)
gainButton.grid(row=3,column=4,padx=5,pady=1)

clearButton = ttk.Button(frame, text='Clear Canvas', command=clearCmd)
clearButton.grid(row=1,column=6,padx=5,pady=1)

outputLabel = Label(frame, text = "UART Output:").grid(row=6,column=3)
outputText = st.ScrolledText(frame,height = 10, font = ("calibre",10))
outputText.grid(row=7,column=2,columnspan = 3,rowspan=6)
outputText.config(spacing1=2,spacing3=2)


def main():
	if(ser.isOpen() == False):
		ser.open()
	
	root.title('Jackson\'s Embedded II GUI')
	root.geometry("1100x800")

	buff = sio.readlines()
	for line in buff:
		print(str(line))
		uartAppend(outputText, str(line))

	root.mainloop()

	ser.close()

if __name__ == '__main__':
	main()
