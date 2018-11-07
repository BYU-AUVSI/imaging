"""
Filename:       Gui.py
File type:      server-side python code
Author:         Jake Charlebois
Created on:     2017-03-19
Modified on:    2017-03-19
Description:    Gui using OpenCV and Tkinter
"""

from tkinter import *
from PIL import Image, ImageTk
from tkinter import messagebox
import cv2


class Gui:

	def __init__(self):

		# Main Tkinter window that we add our elements too
		root = Tk()
		root.title("Auto Tilting Ball Maze")
		root.configure(bg="#007777")

		# Make fullscreen
		root.geometry("{0}x{1}+0+0".format(root.winfo_screenwidth(), root.winfo_screenheight()))

		# Labels such as our thresholds, camera index, and serial port
		Label(root, text="Please Enter Variables", font=("Helvetica", 18), bg="#007777").grid(row=0, columnspan=3)
		Label(root, text="Upper (as B,G,R)", font=("Helvetica", 18), bg="#007777").grid(row=1, column=1)
		Label(root, text="Lower (as B,G,R)", font=("Helvetica", 18), bg="#007777").grid(row=1, column=2)
		Label(root, text="PlaySpace Thresholds", font=("Helvetica", 16), bg="#007777").grid(row=2, sticky=E)
		Label(root, text="Start Thresholds", font=("Helvetica", 16), bg="#007777").grid(row=3, sticky=E)
		Label(root, text="End Thresholds", font=("Helvetica", 16), bg="#007777").grid(row=4, sticky=E)
		Label(root, text="Serial Ports",font=("Helvetica", 16), bg="#007777").grid(row=5, sticky=E)
		Label(root, text="Camera",font=("Helvetica", 16), bg="#007777").grid(row=6, sticky=E)
		Label(root, text="  ", bg="#007777").grid(row=7, sticky=E)

		# Text Fields with defaulted thresholds to save ourselves time
		self.playSpaceEntry1 = Entry(root)
		self.playSpaceEntry1.insert(END, "255,255,255")
		self.playSpaceEntry1.grid(row=2,column=1)
		self.playSpaceEntry2 = Entry(root)
		self.playSpaceEntry2.insert(END, "100,100,100")
		self.playSpaceEntry2.grid(row=2,column=2)
		self.startEntry1 = Entry(root)
		self.startEntry1.insert(END, "255,135,35")
		self.startEntry1.grid(row=3,column=1)
		self.startEntry2 = Entry(root)
		self.startEntry2.insert(END, "70,0,0")
		self.startEntry2.grid(row=3,column=2)
		self.endEntry1 = Entry(root)
		self.endEntry1.insert(END, "235,135,255")
		self.endEntry1.grid(row=4,column=1)
		self.endEntry2 = Entry(root)
		self.endEntry2.insert(END, "150,70,190")
		self.endEntry2.grid(row=4,column=2)

		# Serial Port text field
		self.portNum = Entry(root)
		self.portNum.insert(END, "/dev/ttyUSB0")
		self.portNum.grid(row=5, column=1)

		# Get all available camera's
		camIndices = self.detectNumCameras()
		camIndex = []
		for n in range(camIndices):
			camIndex.append(n)
		camIndex.append("I don't see my camera . . .")

		# Camera Menu
		self.cameraIndex = StringVar(root)
		self.cameraIndex.set("Please select a camera")
		#dropCamera = apply(OptionMenu, (root, self.cameraIndex) + tuple(camIndex))
		#dropCamera.grid(row=6, column=1)
		Button(root, text="Check", font=("Helvetica", 16), command=self.checkCamera).grid(row=6, column=2)#lambda: checkCamera(master))

		# Solve Button
		Button(root, text="                    Solve!                    ", command=self.grabVariables, font=("Helvetica", 16)).grid(row=8, column=0, columnspan=3, rowspan=2)

		#Camera Panel
		imageFrame = Frame(root, width=600, height=500)
		imageFrame.grid(row=0, column=4, rowspan=8)
		self.lmain = Label(imageFrame)
		self.lmain.grid(row=0, column=0)

		# Grid Sizing
		root.columnconfigure(0, weight=1)
		root.rowconfigure(0, weight=1)
		root.columnconfigure(1, weight=1)
		root.rowconfigure(1, weight=1)
		root.columnconfigure(2, weight=1)
		root.rowconfigure(2, weight=1)
		root.columnconfigure(3, weight=1)
		root.rowconfigure(3, weight=1)
		root.rowconfigure(4, weight=1)
		root.rowconfigure(5, weight=1)
		root.rowconfigure(6, weight=1)
		root.rowconfigure(7, weight=1)
		root.rowconfigure(8, weight=1)
		root.rowconfigure(9, weight=1)
		root.columnconfigure(4, weight=1)

		# Info box: reminder to level the playing surface
		messagebox.showinfo("Step 1", "Please manually level the play surface . . .")
		root.mainloop()


	#show_frame insprired by http://stackoverflow.com/questions/16366857/show-webcam-sequence-tkinter
	def show_frame(self):
		_, frame = self.cap.read()
		frame = cv2.flip(frame, 1)
		cv2image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGBA)
		img = Image.fromarray(cv2image)
		imgtk = ImageTk.PhotoImage(image=img)
		self.lmain.imgtk = imgtk
		self.lmain.configure(image=imgtk)
		self.lmain.after(10, self.show_frame)


	# Grabs the variables to be input into our MazeSolver class
	def grabVariables(self):

		# Grab all the variables
		playSpaceUpper = self.playSpaceEntry1.get()
		playSpaceLower = self.playSpaceEntry2.get()
		startUpper = self.startEntry1.get()
		startLower = self.startEntry2.get()
		endUpper = self.endEntry1.get()
		endLower = self.endEntry2.get()
		serialPort = self.portNum.get()
		camIndex = self.cameraIndex.get()

		# Parse our inputs for MazeSolver input
		playSpaceLower = self.parseThreshold(playSpaceLower)
		playSpaceUpper = self.parseThreshold(playSpaceUpper)
		startUpper = self.parseThreshold(startUpper)
		startLower = self.parseThreshold(startLower)
		endUpper = self.parseThreshold(endUpper)
		endLower = self.parseThreshold(endLower)


	# Simple function that returns the number of cameras connected to the 'server'
	def detectNumCameras(self):
		ind = 0
		# Iterates through indexes until we cant find a camera
		while True:
		    vc = cv2.VideoCapture(ind)
		    if (vc.isOpened()):
		        ind += 1
		        vc.release()
		    else:
		        break
		return ind

	# Turns on the camera and continues to run show_frame() to provide a video feed
	# for our Gui. If the camera is already in use it is first released so that our set camera
	# index can be read from instead
	def checkCamera(self):

		try:
			self.cap.release()
		except:
			pass

		self.cap = cv2.VideoCapture(int(self.cameraIndex.get()))

		_, frame = self.cap.read()
		cv2image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGBA)
		img = Image.fromarray(cv2image)
		imgtk = ImageTk.PhotoImage(image=img)
		self.lmain.imgtk = imgtk
		self.lmain.configure(image=imgtk)
		self.lmain.after(10, self.show_frame)


	# Parses the CSV values from the user
	def parseThreshold(self, values):
		strings = values.split(",")
		return [int(string) for string in strings]

# Self call to run when compiled
Gui()
