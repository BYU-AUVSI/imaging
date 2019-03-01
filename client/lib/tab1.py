import sys
sys.path.append('..')

import tkinter as tk
from tkinter import ttk
from lib import tab_tools
import time
import numpy as np
import cv2

"""
# TODO:
    Add zooming feature
    Fix resizing
"""

class Tab1():
    """
    Pull raw images and submit cropped images
    """
    def __init__(self,master,notebook,interface):
        # itialize variables
        self.master = master
        self.n = notebook
        self.interface = interface
        self.initialized = False
        self.resize_counter_tab1 = time.time()

        self.t1_functional = False
        self.x0 = None
        self.y0 = None
        self.x1 = None
        self.y1 = None

        self.imageID = 0
        self.pingServer()
        self.draw_np = np.copy(self.org_np)
        self.img_im = tab_tools.np2im(self.draw_np)
        self.crop_preview_im = self.img_im.copy()
        self.crop_preview_tk = tab_tools.im2tk(self.crop_preview_im)
        self.img_tk = tab_tools.im2tk(self.img_im)
        self.org_width,self.org_height = self.img_im.size
        self.crop_preview_width,self.crop_preview_height = self.img_im.size
        self.cropped = False

        # TAB 1: CROPPING ------------------------------------------------------
        self.tab1 = ttk.Frame(self.n)
        self.n.add(self.tab1, text='Cropping')
        # Allows everthing to be resized
        tk.Grid.rowconfigure(self.tab1,0,weight=7)
        tk.Grid.rowconfigure(self.tab1,1,weight=1)
        tk.Grid.columnconfigure(self.tab1,0,weight=14)
        tk.Grid.columnconfigure(self.tab1,1,weight=1)
        tk.Grid.columnconfigure(self.tab1,2,weight=1)

        self.t1c1i1 = ttk.Label(self.tab1, anchor=tk.CENTER,image=self.img_tk)
        self.t1c1i1.image = self.img_tk
        self.t1c1i1.grid(row=0,column=0,rowspan=3,sticky=tk.N+tk.S+tk.E+tk.W,padx=5,pady=5,ipadx=5,ipady=5)
        self.t1c1i1.bind("<Button-1>",self.mouse_click)
        self.t1c1i1_width = self.t1c1i1.winfo_width()
        self.t1c1i1_height = self.t1c1i1.winfo_height()
        self.crop_preview_img_ratio = 1/7. # ratio between image and crop preview
        self.t1c2i1 = ttk.Label(self.tab1, anchor=tk.CENTER,image=self.crop_preview_tk)
        self.t1c2i1.image = self.crop_preview_tk
        self.t1c2i1.grid(row=0,column=1,columnspan=2,sticky=tk.N+tk.S+tk.E+tk.W,padx=5,pady=5,ipadx=5,ipady=5)
        self.t1c2r1a = ttk.Label(self.tab1, anchor=tk.E, text='Submission Status: ')
        self.t1c2r1a.grid(row=1,column=1,columnspan=1,sticky=tk.N+tk.S+tk.E+tk.W,padx=5,pady=5,ipadx=5,ipady=5)
        self.t1c2r1b = ttk.Label(self.tab1, anchor=tk.W, text='N/A')
        self.t1c2r1b.grid(row=1,column=2,columnspan=1,sticky=tk.N+tk.S+tk.E+tk.W,padx=5,pady=5,ipadx=5,ipady=5)
        self.t1c2b1 = ttk.Button(self.tab1, text="Submit Crop",command=self.submitCropped)
        self.t1c2b1.grid(row=2,column=1,columnspan=2,sticky=tk.N+tk.S+tk.E+tk.W,padx=5,pady=5,ipadx=5,ipady=5)

        self.initialized = True

    def run(self,interface):
        self.interface = interface
        self.resizeEventTab1()
        # setup all keybindings
        self.master.bind("<Right>",self.nextRaw)
        self.master.bind("<Left>",self.previousRaw)
        self.master.unbind("<d>")
        self.master.unbind("<a>")
        self.master.bind("<Configure>",self.resizeEventTab1)
        self.master.bind("<Control-z>",self.undoCrop)
        self.master.bind("<Return>",self.submitCropped)

    def pingServer(self):
        """
        Checks if server is correctly connected

        @rtype:  None
        @return: None
        """
        self.serverConnected = self.interface.ping()
        if self.serverConnected:
            self.org_np = tab_tools.get_image('assets/instructions.jpg')
            self.cropped_np = tab_tools.get_image('assets/classify_instructions.jpg')
        else:
            self.org_np = tab_tools.get_image('assets/server_error.jpg')
            self.cropped_np = tab_tools.get_image('assets/server_error.jpg')

    def resizeEventTab1(self,event=None):
        """
        Resizes pictures on Tab1
        @type  event: event
        @param event: resize window event

        @rtype:  None
        @return: None
        """
        if self.initialized and (time.time()-self.resize_counter_tab1) > 0.050:
            if self.t1c1i1.winfo_width() > 1:
                self.resize_counter_tab1 = time.time()
                self.master.update()
                # main image
                self.t1c1i1_width = self.t1c1i1.winfo_width()
                self.t1c1i1_height = self.t1c1i1.winfo_height()
                self.resized_im = tab_tools.resizeIm(self.img_im,self.org_width,self.org_height,self.t1c1i1_width,self.t1c1i1_height)
                self.t1c1i1_img_width,self.t1c1i1_img_height = self.resized_im.size
                self.img_tk = tab_tools.im2tk(self.resized_im)
                self.t1c1i1.configure(image=self.img_tk)
                # cropped image
                self.crop_preview_resized_im = tab_tools.resizeIm(self.crop_preview_im,self.crop_preview_width,self.crop_preview_height,self.t1c1i1_width*self.crop_preview_img_ratio,self.t1c1i1_height*self.crop_preview_img_ratio)
                self.crop_preview_tk = tab_tools.im2tk(self.crop_preview_resized_im)
                self.t1c2i1.configure(image=self.crop_preview_tk)

    def nextRaw(self,event):
        """
        Requests and displays next raw image

        @type  event: event
        @param event: Right arrow event

        @rtype:  None
        @return: None
        """
        self.pingServer()
        if self.serverConnected:
            query = self.interface.getNextRawImage()
            if query == None:
                self.t1_functional = False
                self.noNextRaw()
            else:
                self.t1_functional = True
                self.imageID = query[1]
                self.org_np = np.array(query[0])
            self.draw_np = np.copy(self.org_np)
            self.img_im = tab_tools.np2im(self.draw_np)
            self.crop_preview_im = self.img_im.copy()
            self.org_width,self.org_height = self.img_im.size
            self.crop_preview_width,self.crop_preview_height = self.crop_preview_im.size
            self.cropped = False
            self.resized_im = tab_tools.resizeIm(self.img_im,self.org_width,self.org_height,self.t1c1i1_width,self.t1c1i1_height)
            self.img_tk = tab_tools.im2tk(self.resized_im)
            self.t1c1i1.configure(image=self.img_tk)
            self.crop_preview_resized_im = tab_tools.resizeIm(self.crop_preview_im,self.crop_preview_width,self.crop_preview_height,self.t1c1i1_width*self.crop_preview_img_ratio,self.t1c1i1_height*self.crop_preview_img_ratio)
            self.crop_preview_tk = tab_tools.im2tk(self.crop_preview_resized_im)
            self.t1c2i1.configure(image=self.crop_preview_tk)
            self.t1c2r1b.configure(text="unsubmitted",foreground="red")
            # reset crop points to none
            self.x0 = None
            self.y0 = None
            self.x1 = None
            self.y1 = None


    def previousRaw(self,event):
        """
        Requests and displays previous raw image

        @type  event: event
        @param event: Left arrow event

        @rtype:  None
        @return: None
        """
        self.pingServer()
        if self.serverConnected:
            query = self.interface.getPrevRawImage()
            if query == None:
                self.noPreviousRaw()
                self.t1_functional = False
            else:
                self.t1_functional = True
                self.imageID = query[1]
                self.org_np = np.array(query[0]) #tab_tools.get_image('frame0744.jpg')
            self.draw_np = np.copy(self.org_np)
            self.img_im = tab_tools.np2im(self.draw_np)
            self.crop_preview_im = self.img_im.copy()
            self.crop_preview_tk = tab_tools.im2tk(self.crop_preview_im)
            self.org_width,self.org_height = self.img_im.size
            self.crop_preview_width,self.crop_preview_height = self.img_im.size
            self.cropped = False
            self.resized_im = tab_tools.resizeIm(self.img_im,self.org_width,self.org_height,self.t1c1i1_width,self.t1c1i1_height)
            self.img_tk = tab_tools.im2tk(self.resized_im)
            self.t1c1i1.configure(image=self.img_tk)
            self.crop_preview_resized_im = tab_tools.resizeIm(self.crop_preview_im,self.crop_preview_width,self.crop_preview_height,self.t1c1i1_width*self.crop_preview_img_ratio,self.t1c1i1_height*self.crop_preview_img_ratio)
            self.crop_preview_tk = tab_tools.im2tk(self.crop_preview_resized_im)
            self.t1c2i1.configure(image=self.crop_preview_tk)
            self.t1c2r1b.configure(text="unsubmitted",foreground="red")
            # reset crop points to none
            self.x0 = None
            self.y0 = None
            self.x1 = None
            self.y1 = None


    def submitCropped(self,event=None):
        """
        Submits cropped image to server

        @type  event: event
        @param event: Enter press or button press event

        @rtype:  None
        @return: None
        """
        if self.t1_functional:
            self.interface.postCroppedImage(self.imageID,self.crop_preview_im,[self.cx0,self.cy0],[self.cx1,self.cy1])
            self.t1c2r1b.configure(text="submitted",foreground="green")


    def undoCrop(self,event=None):
        """
        Undoes crop and resets the raw image

        @type  event: event
        @param event: Ctrl + Z event

        @rtype:  None
        @return: None
        """
        if event != None:
            self.x0 = None
            self.y0 = None
            self.x1 = None
            self.y1 = None
        self.draw_np = np.copy(self.org_np)
        self.img_im = tab_tools.np2im(self.draw_np)
        self.resized_im = tab_tools.resizeIm(self.img_im,self.org_width,self.org_height,self.t1c1i1_width,self.t1c1i1_height)
        self.img_tk = tab_tools.im2tk(self.resized_im)
        self.t1c1i1.configure(image=self.img_tk)
        self.crop_preview_im = self.img_im.copy()
        self.crop_preview_width,self.crop_preview_height = self.crop_preview_im.size
        self.crop_preview_resized_im = tab_tools.resizeIm(self.crop_preview_im,self.crop_preview_width,self.crop_preview_height,self.t1c1i1_width*self.crop_preview_img_ratio,self.t1c1i1_height*self.crop_preview_img_ratio)
        self.crop_preview_tk = tab_tools.im2tk(self.crop_preview_resized_im)
        self.t1c2i1.configure(image=self.crop_preview_tk)


    def mouse_click(self,event):
        """
        Saves pixel location of where on the image the mouse clicks
        @type  event: event
        @param event: mouse event

        @rtype:  None
        @return: None
        """
        self.t1c1i1.bind("<ButtonRelease-1>",self.mouse_release)
        self.t1c1i1.bind("<Motion>",self.mouse_move)
        # calculate offset between container size and image size
        self.offset_x = int((self.t1c1i1_width - self.t1c1i1_img_width )/2.0)
        self.offset_y = int((self.t1c1i1_height - self.t1c1i1_img_height)/2.0)
        x0 = event.x - self.offset_x
        y0 = event.y - self.offset_y

        self.new_crop = True
        # check if there has been a rectangle drawn yet
        if self.x0 != None and self.x1 != None:
            # check if clicked inside the previous rectangle
            if self.x0 < self.x1 and self.y0 < self.y1:
                if x0 > self.x0 and x0 < self.x1 and y0 > self.y0 and y0 < self.y1:
                    self.new_crop = False
            if self.x0 < self.x1 and self.y0 > self.y1:
                if x0 > self.x0 and x0 < self.x1 and y0 < self.y0 and y0 > self.y1:
                    self.new_crop = False
            if self.x0 > self.x1 and self.y0 < self.y1:
                if x0 < self.x0 and x0 > self.x1 and y0 > self.y0 and y0 < self.y1:
                    self.new_crop = False
            if self.x0 > self.x1 and self.y0 > self.y1:
                if x0 < self.x0 and x0 > self.x1 and y0 < self.y0 and y0 > self.y1:
                    self.new_crop = False

        if self.new_crop:
            self.x0 = x0
            self.y0 = y0
            if self.x0 > self.resized_im.size[0]:
                self.x0 = self.resized_im.size[0]
            elif self.x0 < 0:
                self.x0 = 0
            if self.y0 > self.resized_im.size[1]:
                self.y0 = self.resized_im.size[1]
            elif self.y0 < 0:
                self.y0 = 0
        else:
            # it's a pan, not a new crop
            self.pan_x0 = x0
            self.pan_y0 = y0


    def mouse_move(self,event):
        """
        Gets pixel location of where the mouse is moving and show rectangle for crop preview
        @type  event: event
        @param event: mouse event

        @rtype:  None
        @return: None
        """
        self.t1c1i1.bind("<ButtonRelease-1>",self.mouse_release)
        disp_width,disp_height = self.resized_im.size
        # ratio between full-size image and displayed image
        self.sr = (self.org_width/disp_width + self.org_height/disp_height)/2.0
        self.draw_np = np.copy(self.org_np)

        x1 = event.x - self.offset_x
        y1 = event.y - self.offset_y
        # prevent from going out of bounds
        if x1 > self.resized_im.size[0]:
            x1 = self.resized_im.size[0]
        elif x1 < 0:
            x1 = 0
        if y1 > self.resized_im.size[1]:
            y1 = self.resized_im.size[1]
        elif y1 < 0:
            y1 = 0
        if self.new_crop:
            self.x1 = x1
            self.y1 = y1
            cv2.rectangle(self.draw_np,(int(self.sr*self.x0),int(self.sr*self.y0)),(int(self.sr*self.x1),int(self.sr*self.y1)),(255,0,0),2)
        else:
            self.pan_x1 = x1
            self.pan_y1 = y1
            xdif = int((self.pan_x1 - self.pan_x0))
            ydif = int((self.pan_y1 - self.pan_y0))
            self.x0_hat = self.x0 + xdif
            self.y0_hat = self.y0 + ydif
            self.x1_hat = self.x1 + xdif
            self.y1_hat = self.y1 + ydif

            # prevent panning out of bounds
            if self.x0_hat < self.x1_hat:
                if self.x0_hat < 0:
                    self.x0_hat = 0
                    self.x1_hat = np.abs(self.x1-self.x0)
                elif self.x1_hat > self.resized_im.size[0]:
                    self.x0_hat = self.resized_im.size[0]-np.abs(self.x1-self.x0)
                    self.x1_hat = self.resized_im.size[0]
            else:
                if self.x1_hat < 0:
                    self.x1_hat = 0
                    self.x0_hat = np.abs(self.x1-self.x0)
                elif self.x0_hat > self.resized_im.size[0]:
                    self.x1_hat = self.resized_im.size[0]-np.abs(self.x1-self.x0)
                    self.x0_hat = self.resized_im.size[0]
            if self.y0_hat < self.y1_hat:
                if self.y0_hat < 0:
                    self.y0_hat = 0
                    self.y1_hat = np.abs(self.y1-self.y0)
                elif self.y1_hat > self.resized_im.size[1]:
                    self.y0_hat = self.resized_im.size[1]-np.abs(self.y1-self.y0)
                    self.y1_hat = self.resized_im.size[1]
            else:
                if self.y1_hat < 0:
                    self.y1_hat = 0
                    self.y0_hat = np.abs(self.x1-self.x0)
                elif self.y0_hat > self.resized_im.size[1]:
                    self.y1_hat = self.resized_im.size[1]-np.abs(self.y1-self.y0)
                    self.y0_hat = self.resized_im.size[1]

            cv2.rectangle(self.draw_np,(int(self.sr*self.x0_hat),int(self.sr*self.y0_hat)),(int(self.sr*self.x1_hat),int(self.sr*self.y1_hat)),(255,0,0),2)
            cv2.line(self.draw_np,(int(self.sr*self.pan_x0),int(self.sr*self.pan_y0)),(int(self.sr*self.pan_x1),int(self.sr*self.pan_y1)),(45,255,255),2)

        self.img_im = tab_tools.np2im(self.draw_np)
        self.resized_im = tab_tools.resizeIm(self.img_im,self.org_width,self.org_height,self.t1c1i1_width,self.t1c1i1_height)
        self.img_tk = tab_tools.im2tk(self.resized_im)
        self.t1c1i1.configure(image=self.img_tk)

    def mouse_release(self,event):
        """
        Saves pixel location of where the mouse clicks and creates crop preview
        @type  event: event
        @param event: mouse event

        @rtype:  None
        @return: None
        """
        if self.cropped:
            self.undoCrop()
        self.t1c1i1.unbind("<Motion>")
        self.t1c1i1.unbind("<ButtonRelease-1>")

        if not(self.new_crop):
            # save hat values as the new values
            self.x0 = self.x0_hat
            self.y0 = self.y0_hat
            self.x1 = self.x1_hat
            self.y1 = self.y1_hat

        if self.x0 != self.x1 or self.y0 != self.y1:
            cv2.rectangle(self.draw_np,(int(self.sr*self.x0),int(self.sr*self.y0)),(int(self.sr*self.x1),int(self.sr*self.y1)),(255,0,0),2)
            self.cropImage(int(self.sr*self.x0),int(self.sr*self.y0),int(self.sr*self.x1),int(self.sr*self.y1))
            self.img_im = tab_tools.np2im(self.draw_np)
            self.resized_im = tab_tools.resizeIm(self.img_im,self.org_width,self.org_height,self.t1c1i1_width,self.t1c1i1_height)
            self.img_tk = tab_tools.im2tk(self.resized_im)
            self.t1c1i1.configure(image=self.img_tk)
            # Crop Image
            self.cropped = True
            self.t1c2r1b.configure(text="unsubmitted",foreground="red")

    def cropImage(self,x0,y0,x1,y1):
        """
        Crops raw image
        @type  x0: integer
        @param x0: pixel x location of first click

        @type  y0: integer
        @param y0: pixel y location of first click

        @type  x1: integer
        @param x1: pixel x location of second click

        @type  y1: integer
        @param y1: pixel y location of second click

        @rtype:  None
        @return: None
        """
        if x0 < x1:
            self.cx0 = x0
            self.cx1 = x1
        else:
            self.cx0 = x1
            self.cx1 = x0
        if y0 < y1:
            self.cy0 = y0
            self.cy1 = y1
        else:
            self.cy0 = y1
            self.cy1 = y0
        self.crop_preview_im = self.crop_preview_im.crop((self.cx0,self.cy0,self.cx1,self.cy1))
        self.crop_preview_width,self.crop_preview_height = self.crop_preview_im.size
        self.crop_preview_resized_im = tab_tools.resizeIm(self.crop_preview_im,self.crop_preview_width,self.crop_preview_height,self.t1c1i1_width*self.crop_preview_img_ratio,self.t1c1i1_height*self.crop_preview_img_ratio)
        self.crop_preview_tk = tab_tools.im2tk(self.crop_preview_resized_im)
        self.t1c2i1.configure(image=self.crop_preview_tk)

    def noNextRaw(self):
        """
        Checks if server is correctly connected

        @rtype:  None
        @return: None
        """
        self.serverConnected = self.interface.ping()
        if self.serverConnected:
            self.org_np = tab_tools.get_image('assets/noNextRaw.jpg')
        else:
            self.org_np = tab_tools.get_image('assets/server_error.jpg')
        self.t1c2r1b.configure(text="N/A",foreground="#636363")

    def noPreviousRaw(self):
        """
        Checks if server is correctly connected

        @rtype:  None
        @return: None
        """
        self.serverConnected = self.interface.ping()
        if self.serverConnected:
            self.org_np = tab_tools.get_image('assets/noPreviousRaw.jpg')
        else:
            self.org_np = tab_tools.get_image('assets/server_error.jpg')
        self.t1c2r1b.configure(text="N/A",foreground="#636363")
