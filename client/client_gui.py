'''
Authors: D. Knowles, B. McBride, T. Miller

Prereqs:
python 3
sudo apt install python3-tk
pip3 install Pillow, opencv-python, ttkthemes
'''

'''
TODO:
Integration:
    Next Cropped
    Prev Cropped
    Submit classification
    Geolocation
All:
    *possible threading behind the scenese to autosize other tabs
Tab0:
    add error handling if entries aren't in the right format
    add error handling if not connected to correct wifi
Tab1:
    Add zooming feature
    Add panning feature
    Fix bug of initial sizing
    Fix how crop sizes
Tab2:
    Change disable color
    Rotate image accordingex to heading
    Add classified Targets
    Add classification queue
Tab3:
    Make everything
    Manual tender as well

KNOWN BUGS:
    Weird exit is bound to an event (doesn't do anything) when rerun in iPython3
    If you click, but don't drag on the crop screen, you still see rectangle
    When you click on the 2nd tab right at the beginning, and then use the left/intruder_height
        buttons, it moves one tab, then unbinds like it's supposed to.
'''

import tkinter as tk
from tkinter import ttk
from ttkthemes import ThemedStyle
from PIL import Image,ImageTk
import cv2
import numpy as np
import time
import client_rest
import time
import sys


class GuiClass(tk.Frame):
    """
    Graphical User Interface for 2019 AUVSI competition
    Tab 0: Setting for setting up the server_error
    Tab 1: Pull raw images and submit cropped images
    Tab 2: Pull cropped iamges and submit classification for images
    Tab 3: Display results for manual and autonomous classification

    @type tk.Frame: nothing
    @param tk.Frame: nothing
    """
    def __init__(self,master=None):
        """
        initialization for Gui Class

        @rtype:  None
        @return: None
        """


        tk.Frame.__init__(self,master=None)
        self.master = master # gui master handle
        try:
            self.master.attributes('-zoomed', True) # maximizes screen
        except (Exception) as e:
            w, h = root.winfo_screenwidth(), root.winfo_screenheight()
            root.geometry("%dx%d+0+0" % (w, h))
        self.master.title("BYU AUVSI COMPETITION 2019")

        self.n = ttk.Notebook(self.master) # create tabs
        self.n.pack(fill=tk.BOTH, expand=1) # expand across space
        tk.Grid.rowconfigure(self.master,0,weight=1)
        tk.Grid.columnconfigure(self.master,0,weight=1)

        # itialize variables
        #self.default_host = '192.168.1.48'
        self.default_host = '127.0.0.5'
        self.default_port = '5000'
        self.default_idnum = 50
        self.default_debug = False
        self.imageID = 0
        self.interface = client_rest.ImagingInterface(host=self.default_host,port=self.default_port,numIdsStored=self.default_idnum,isDebug=self.default_debug)
        self.initialized = False
        self.pingServer()
        self.draw_np = np.copy(self.org_np)
        self.img_im = self.np2im(self.draw_np)
        self.crop_preview_im = self.img_im.copy()
        self.crop_preview_tk = self.im2tk(self.crop_preview_im)
        self.img_tk = self.im2tk(self.img_im)
        self.org_width,self.org_height = self.img_im.size
        self.crop_preview_width,self.crop_preview_height = self.img_im.size
        self.resize_counter_tab0 = time.time()
        self.resize_counter_tab1 = time.time()
        self.resize_counter_tab2 = time.time()
        self.cropped = False
        self.cropped_im = self.np2im(self.cropped_np)
        self.cropped_width,self.cropped_height = self.cropped_im.size
        self.cropped_tk = self.im2tk(self.cropped_im)


        # Tab 0: SETTINGS ------------------------------------------------------
        self.tab0 = ttk.Frame(self.n)
        self.n.add(self.tab0, text='Settings')
        for x in range(6):
            tk.Grid.columnconfigure(self.tab0,x,weight=1)
        for y in range(10):
            tk.Grid.rowconfigure(self.tab0,y,weight=1)
        # Column One
        self.t0c1l1 = ttk.Label(self.tab0, anchor=tk.CENTER, text='                               ')
        self.t0c1l1.grid(row=0,column=0,sticky=tk.N+tk.S+tk.E+tk.W,padx=5,pady=5,ipadx=5,ipady=5)
        self.t0c1l2 = ttk.Label(self.tab0, anchor=tk.CENTER, text='                               ')
        self.t0c1l2.grid(row=0,column=1,sticky=tk.N+tk.S+tk.E+tk.W,padx=5,pady=5,ipadx=5,ipady=5)

        # Column Two
        #self.t0sep12 = ttk.Separator(self.tab0, orient=tk.VERTICAL)
        #self.t0sep12.grid(row=0, column=2, rowspan=10,sticky=tk.N+tk.S+tk.E+tk.W, pady=5)
        self.logo_np = self.get_image('logo.png')
        self.logo_im = self.np2im(self.logo_np)
        self.logo_width,self.logo_height = self.logo_im.size
        self.logo_tk = self.im2tk(self.logo_im)

        self.t0c2i1 = ttk.Label(self.tab0, anchor=tk.CENTER,image=self.logo_tk)
        self.t0c2i1.image = self.logo_tk
        self.t0c2i1.grid(row=0,column=2,rowspan=5,columnspan=2,sticky=tk.N+tk.S+tk.E+tk.W,padx=5,pady=5,ipadx=5,ipady=5)
        self.t0c2l1 = ttk.Label(self.tab0, anchor=tk.E, text='Host:')
        self.t0c2l1.grid(row=5,column=2,sticky=tk.N+tk.S+tk.E+tk.W,padx=5,pady=5,ipadx=5,ipady=5)
        self.t0c2host = tk.StringVar()
        self.t0c2host.set(self.default_host)
        self.t0c2l2 = ttk.Entry(self.tab0,textvariable=self.t0c2host)
        self.t0c2l2.grid(row=5,column=3,sticky=tk.N+tk.S+tk.W,padx=5,pady=5,ipadx=5,ipady=5)
        self.t0c2l3 = ttk.Label(self.tab0, anchor=tk.E, text='Port:')
        self.t0c2l3.grid(row=6,column=2,sticky=tk.N+tk.E+tk.S+tk.W,padx=5,pady=5,ipadx=5,ipady=5)
        self.t0c2port = tk.StringVar()
        self.t0c2port.set(self.default_port)
        self.t0c2l4 = ttk.Entry(self.tab0,textvariable=self.t0c2port)
        self.t0c2l4.grid(row=6,column=3,sticky=tk.N+tk.S+tk.W,padx=5,pady=5,ipadx=5,ipady=5)
        self.t0c2l5 = ttk.Label(self.tab0, anchor=tk.E, text='Number of IDs Stored:')
        self.t0c2l5.grid(row=7,column=2,sticky=tk.N+tk.S+tk.E+tk.W,padx=5,pady=5,ipadx=5,ipady=5)
        self.t0c2ids = tk.StringVar()
        self.t0c2ids.set(self.default_idnum)
        self.t0c2l6 = ttk.Entry(self.tab0,textvariable=self.t0c2ids)
        self.t0c2l6.grid(row=7,column=3,sticky=tk.N+tk.S+tk.W,padx=5,pady=5,ipadx=5,ipady=5)
        self.t0c2l7 = ttk.Label(self.tab0, anchor=tk.E, text='Debug Mode:')
        self.t0c2l7.grid(row=8,column=2,sticky=tk.N+tk.S+tk.E+tk.W,padx=5,pady=5,ipadx=5,ipady=5)
        self.t0c2debug = tk.IntVar()
        self.t0c2l8 = ttk.Radiobutton(self.tab0,text='True',value=0,variable=self.t0c2debug)
        self.t0c2l8.grid(row=8,column=3,sticky=tk.N+tk.S+tk.W,padx=5,pady=5,ipadx=5,ipady=5)
        self.t0c2l8b = ttk.Radiobutton(self.tab0,text='False',value=1,variable=self.t0c2debug)
        self.t0c2l8b.grid(row=8,column=3,sticky=tk.N+tk.S,padx=5,pady=5,ipadx=5,ipady=5)
        if not self.default_debug:
            self.t0c2debug.set(1)
        self.t0c2l9 = ttk.Button(self.tab0, text="Apply Settings",command=self.updateSettings)
        self.t0c2l9.grid(row=9,column=2,columnspan=2,sticky=tk.N+tk.S,padx=5,pady=5,ipadx=5,ipady=5)
        # Column Three
        #self.t0sep23 = ttk.Separator(self.tab0, orient=tk.VERTICAL)
        #self.t0sep23.grid(row=0, column=4, rowspan=10,sticky=tk.N+tk.S+tk.E+tk.W, pady=5)
        self.t0c3l1 = ttk.Label(self.tab0, anchor=tk.CENTER, text='                                   ')
        self.t0c3l1.grid(row=0,column=4,sticky=tk.N+tk.S+tk.E+tk.W,padx=5,pady=5,ipadx=5,ipady=5)
        self.t0c3l2 = ttk.Label(self.tab0, anchor=tk.CENTER, text='                                   ')
        self.t0c3l2.grid(row=0,column=5,sticky=tk.N+tk.S+tk.E+tk.W,padx=5,pady=5,ipadx=5,ipady=5)

        # TAB 1: CROPPING ------------------------------------------------------
        self.tab1 = ttk.Frame(self.n)
        self.n.add(self.tab1, text='Cropping')
        # Allows everthing to be resized
        tk.Grid.rowconfigure(self.tab1,0,weight=7)
        tk.Grid.rowconfigure(self.tab1,1,weight=1)
        tk.Grid.columnconfigure(self.tab1,0,weight=7)
        tk.Grid.columnconfigure(self.tab1,1,weight=1)

        self.t1c1i1 = ttk.Label(self.tab1, anchor=tk.CENTER,image=self.img_tk)
        self.t1c1i1.image = self.img_tk
        self.t1c1i1.grid(row=0,column=0,rowspan=2,sticky=tk.N+tk.S+tk.E+tk.W,padx=5,pady=5,ipadx=5,ipady=5)
        self.t1c1i1.bind("<Button-1>",self.mouse_click)
        self.t1c1i1_width = self.t1c1i1.winfo_width()
        self.t1c1i1_height = self.t1c1i1.winfo_height()

        self.crop_preview_img_ratio = 1/7.
        self.t1c2i1 = ttk.Label(self.tab1, anchor=tk.CENTER,image=self.crop_preview_tk)
        self.t1c2i1.image = self.crop_preview_tk
        self.t1c2i1.grid(row=0,column=1,sticky=tk.N+tk.S+tk.E+tk.W,padx=5,pady=5,ipadx=5,ipady=5)
        #self.t1c2i1_width = self.t1c2i1.winfo_width()
        #self.t1c2i1_height = self.t1c2i1.winfo_height()

        self.t1c2b1 = ttk.Button(self.tab1, text="Submit Crop",command=self.submitCropped)
        self.t1c2b1.grid(row=1,column=1,sticky=tk.N+tk.S+tk.E+tk.W,padx=5,pady=5,ipadx=5,ipady=5)



        # TAB 2: CLASSIFICATION ------------------------------------------------
        self.tab2 = ttk.Frame(self.n)   # second page
        self.n.add(self.tab2, text='Classification')

        for x in range(16):
            tk.Grid.columnconfigure(self.tab2,x,weight=1)
        for y in range(50):
            tk.Grid.rowconfigure(self.tab2,y,weight=1)

        # Column One
        self.t2c1title = ttk.Label(self.tab2, anchor=tk.CENTER, text='Classification Queue')
        self.t2c1title.grid(row=0,column=0,columnspan=4,sticky=tk.N+tk.S+tk.E+tk.W,padx=5,pady=5,ipadx=5,ipady=5)

        # Column Two
        self.t2sep12 = ttk.Separator(self.tab2, orient=tk.VERTICAL)
        self.t2sep12.grid(row=0, column=4, rowspan=50,sticky=tk.N+tk.S+tk.E+tk.W, pady=5)
        self.t2c2title = ttk.Label(self.tab2, anchor=tk.CENTER, text='Classification')
        self.t2c2title.grid(row=0,column=4,columnspan=8,sticky=tk.N+tk.S+tk.E+tk.W,padx=5,pady=5,ipadx=5,ipady=5)

        self.t2c2i1 = ttk.Label(self.tab2, anchor=tk.CENTER,image=self.cropped_tk)
        self.t2c2i1.image = self.cropped_tk
        self.t2c2i1.grid(row=2,column=4,rowspan=38,columnspan=8,sticky=tk.N+tk.S+tk.E+tk.W,padx=5,pady=5,ipadx=5,ipady=5)

        self.t2c2l1 = ttk.Label(self.tab2, anchor=tk.CENTER, text='Shape')
        self.t2c2l1.grid(row=40,column=4,columnspan=2,rowspan=2,sticky=tk.N+tk.S+tk.E+tk.W,padx=5,pady=5,ipadx=5,ipady=5)
        shape_options = ('circle','semicircle','quarter_circle','triangle','square','rectangle','trapezoid','pentagon','hexagon','heptagon','octagon','star','cross')
        self.t2c2l2_var = tk.StringVar(self.master)
        #self.t2c2l2_var.set('circle')
        self.t2c2l2 = ttk.OptionMenu(self.tab2,self.t2c2l2_var,shape_options[0],*shape_options)
        self.t2c2l2.grid(row=42,column=4,columnspan=2,rowspan=2,sticky=tk.N+tk.S+tk.E+tk.W,padx=5,pady=5,ipadx=5,ipady=5)
        self.t2c2l3 = ttk.Label(self.tab2, anchor=tk.CENTER, text='Alphanumeric')
        self.t2c2l3.grid(row=40,column=6,columnspan=2,rowspan=2,sticky=tk.N+tk.S+tk.E+tk.W,padx=5,pady=5,ipadx=5,ipady=5)
        self.t2c2l4 = ttk.Entry(self.tab2)
        self.t2c2l4.grid(row=42,column=6,columnspan=2,rowspan=2,sticky=tk.N+tk.S+tk.E+tk.W,padx=5,pady=5,ipadx=5,ipady=5)
        self.t2c2l5 = ttk.Label(self.tab2, anchor=tk.CENTER, text='Orientation')
        self.t2c2l5.grid(row=40,column=8,columnspan=4,rowspan=2,sticky=tk.N+tk.S+tk.E+tk.W,padx=5,pady=5,ipadx=5,ipady=5)
        orientation_options = ('N','NE','E','SE','S','SW','W','NW')
        self.t2c2l6_var = tk.StringVar(self.master)
        #self.t2c2l6_var.set('N')
        self.t2c2l6 = ttk.OptionMenu(self.tab2,self.t2c2l6_var,orientation_options[0],*orientation_options)
        self.t2c2l6.grid(row=42,column=8,columnspan=4,rowspan=2,sticky=tk.N+tk.S+tk.E+tk.W,padx=5,pady=5,ipadx=5,ipady=5)
        self.t2c2l9 = ttk.Label(self.tab2, anchor=tk.CENTER, text='Background Color')
        self.t2c2l9.grid(row=44,column=4,columnspan=2,rowspan=2,sticky=tk.N+tk.S+tk.E+tk.W,padx=5,pady=5,ipadx=5,ipady=5)
        color_options = ('white','black','gray','red','blue','green','yellow','purple','brown','orange')
        self.t2c2l10_var = tk.StringVar(self.master)
        #self.t2c2l10_var.set('white')
        self.t2c2l10 = ttk.OptionMenu(self.tab2,self.t2c2l10_var,color_options[0],*color_options)
        self.t2c2l10.grid(row=46,column=4,columnspan=2,rowspan=2,sticky=tk.N+tk.S+tk.E+tk.W,padx=5,pady=5,ipadx=5,ipady=5)
        self.t2c2l11 = ttk.Label(self.tab2, anchor=tk.CENTER, text='Alphanumeric Color')
        self.t2c2l11.grid(row=44,column=6,columnspan=2,rowspan=2,sticky=tk.N+tk.S+tk.E+tk.W,padx=5,pady=5,ipadx=5,ipady=5)
        self.t2c2l12_var = tk.StringVar(self.master)
        #self.t2c2l12_var.set('white')
        self.t2c2l12 = ttk.OptionMenu(self.tab2,self.t2c2l12_var,color_options[0],*color_options)
        self.t2c2l12.grid(row=46,column=6,columnspan=2,rowspan=2,sticky=tk.N+tk.S+tk.E+tk.W,padx=5,pady=5,ipadx=5,ipady=5)
        self.t2c2l13 = ttk.Label(self.tab2, anchor=tk.CENTER, text='Target Type')
        self.t2c2l13.grid(row=44,column=8,columnspan=2,rowspan=2,sticky=tk.N+tk.S+tk.E+tk.W,padx=5,pady=5,ipadx=5,ipady=5)
        self.t2c2l14_var = tk.StringVar(self.master)
        target_options = ('standard','emergent','off-axis')
        #self.t2c2l14_var.set('standard')
        self.t2c2l14 = ttk.OptionMenu(self.tab2,self.t2c2l14_var,target_options[0],*target_options)
        self.t2c2l14.grid(row=46,column=8,columnspan=2,rowspan=2,sticky=tk.N+tk.S+tk.E+tk.W,padx=5,pady=5,ipadx=5,ipady=5)

        self.t2c2l14_var.trace("w",self.disableEmergentDescription)

        self.t2c2l15 = ttk.Label(self.tab2, anchor=tk.CENTER, text='Emergent Description')
        self.t2c2l15.grid(row=44,column=10,columnspan=2,rowspan=2,sticky=tk.N+tk.S+tk.E+tk.W,padx=5,pady=5,ipadx=5,ipady=5)
        self.t2c2l16 = ttk.Entry(self.tab2)
        self.t2c2l16.grid(row=46,column=10,columnspan=2,rowspan=2,sticky=tk.N+tk.S+tk.E+tk.W,padx=5,pady=5,ipadx=5,ipady=5)
        self.t2c2l17 = ttk.Button(self.tab2, text="Submit Classification",command=self.submitClassification)
        self.t2c2l17.grid(row=48,column=4,columnspan=8,rowspan=2,sticky=tk.N+tk.S+tk.E+tk.W,padx=5,pady=5,ipadx=5,ipady=5)
        self.disableEmergentDescription()

        # Column Three
        self.t2sep23 = ttk.Separator(self.tab2, orient=tk.VERTICAL)
        self.t2sep23.grid(row=0, column=12, rowspan=50,sticky=tk.N+tk.S+tk.E+tk.W, pady=5)
        self.t2c2title = ttk.Label(self.tab2, anchor=tk.CENTER, text='Classified Targets')
        self.t2c2title.grid(row=0,column=12,columnspan=4,sticky=tk.N+tk.S+tk.E+tk.W,padx=5,pady=5,ipadx=5,ipady=5)



        # TAB 3: AUTONOMOUS TENDER ------------------------------------------------
        self.tab3 = ttk.Frame(self.n)
        self.n.add(self.tab3, text='Autonomous Tender')

        # Done with initialization
        self.initialized = True

        # KEY BINDINGS ------------------------------------------------------------
        self.master.bind("<<NotebookTabChanged>>",self.tabChanged)



    def get_image(self,path):
        """
        Reads in an image from folder on computer

        @type  path: file path
        @param path: the file path to where the image is located

        @rtype:  Numpy image array
        @return: Numpy array of selected image
        """
        image_np = cv2.imread(path)
        image_np = cv2.cvtColor(image_np,cv2.COLOR_BGR2RGB)
        return image_np

    def np2im(self,image):
        """
        Converts from numpy array to PIL image
        @type image: Numpy image array
        @param image: Numpy array of selected image

        @rtype:  PIL image
        @return: PIL image of numpy array
        """
        image_im = Image.fromarray(image)
        return image_im

    def im2tk(self,image):
        """
        Converts from PIL image to TK image
        @type image: PIL image
        @param image: PIL image of numpy array

        @rtype:  TK image
        @return: TK image of PIL image
        """
        image_tk = ImageTk.PhotoImage(image)
        return image_tk

    def mouse_click(self,event):
        """
        Saves pixel location of where on the image the mouse clicks
        @type  event: event
        @param event: mouse event

        @rtype:  None
        @return: None
        """
        self.t1c1i1.bind("<Motion>",self.mouse_move)
        self.offset_x = int((self.t1c1i1_width - self.t1c1i1_img_width )/2.0)
        self.offset_y = int((self.t1c1i1_height - self.t1c1i1_img_height)/2.0)
        self.x0 = event.x - self.offset_x
        self.y0 = event.y - self.offset_y

    def mouse_move(self,event):
        """
        Gets pixel location of where the mouse is moving and show rectangle for crop preview
        @type  event: event
        @param event: mouse event

        @rtype:  None
        @return: None
        """
        self.t1c1i1.bind("<ButtonRelease-1>",self.mouse_release)
        self.x1 = event.x - self.offset_x
        self.y1 = event.y - self.offset_y
        disp_width,disp_height = self.resized_im.size
        sr = (self.org_width/disp_width + self.org_height/disp_height)/2.0
        self.draw_np = np.copy(self.org_np)
        cv2.rectangle(self.draw_np,(int(sr*self.x0),int(sr*self.y0)),(int(sr*self.x1),int(sr*self.y1)),(255,0,0),2)
        self.img_im = self.np2im(self.draw_np)
        self.resized_im = self.resizeIm(self.img_im,self.org_width,self.org_height,self.t1c1i1_width,self.t1c1i1_height)
        self.img_tk = self.im2tk(self.resized_im)
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
        self.x1 = event.x - self.offset_x
        self.y1 = event.y - self.offset_y
        disp_width,disp_height = self.resized_im.size
        sr = (self.org_width/disp_width + self.org_height/disp_height)/2.0
        self.draw_np = np.copy(self.org_np)
        cv2.rectangle(self.draw_np,(int(sr*self.x0),int(sr*self.y0)),(int(sr*self.x1),int(sr*self.y1)),(255,0,0),2)
        self.img_im = self.np2im(self.draw_np)
        self.resized_im = self.resizeIm(self.img_im,self.org_width,self.org_height,self.t1c1i1_width,self.t1c1i1_height)
        self.img_tk = self.im2tk(self.resized_im)
        self.t1c1i1.configure(image=self.img_tk)
        # Crop Image
        self.cropImage(int(sr*self.x0),int(sr*self.y0),int(sr*self.x1),int(sr*self.y1))
        self.cropped = True

    def close_window(self,event):
        """
        Closes gui safely
        @type  event: event
        @param event: ESC event

        @rtype:  None
        @return: None
        """
        self.master.destroy()
        sys.exit()

    def resizeEventTab0(self,event=None):
        """
        Resizes picture on Tab0
        @type  event: event
        @param event: resize window event

        @rtype:  None
        @return: None
        """
        if self.initialized and (time.time()-self.resize_counter_tab0) > 0.050:
            if self.t0c2i1.winfo_width() > 1:
                self.resize_counter_tab0 = time.time()
                self.master.update()
                t0c2i1_width = self.t0c2i1.winfo_width()
                t0c2i1_height = self.t0c2i1.winfo_height()
                logoW, logoH = self.logo_im.size
                self.logo_resized_im = self.resizeIm(self.logo_im, logoW, logoH, t0c2i1_width, t0c2i1_height)
                self.logo_tk = self.im2tk(self.logo_resized_im)
                self.t0c2i1.configure(image=self.logo_tk)

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
                self.resized_im = self.resizeIm(self.img_im,self.org_width,self.org_height,self.t1c1i1_width,self.t1c1i1_height)
                self.t1c1i1_img_width,self.t1c1i1_img_height = self.resized_im.size
                self.img_tk = self.im2tk(self.resized_im)
                self.t1c1i1.configure(image=self.img_tk)
                # cropped image
                #self.t1c2i1_width = self.t1c2i1.winfo_width()
                #self.t1c2i1_height = self.t1c2i1.winfo_height()
                self.crop_preview_resized_im = self.resizeIm(self.crop_preview_im,self.crop_preview_width,self.crop_preview_height,self.t1c1i1_width*self.crop_preview_img_ratio,self.t1c1i1_height*self.crop_preview_img_ratio)
                #self.t1c2i1_width,self.t1c2i1_height = self.crop_resized_im.size
                self.crop_preview_tk = self.im2tk(self.crop_preview_resized_im)
                self.t1c2i1.configure(image=self.crop_preview_tk)

    def resizeEventTab2(self,event=None):
        """
        Resizes picture on Tab2
        @type  event: event
        @param event: resize window event

        @rtype:  None
        @return: None
        """
        if self.initialized and (time.time()-self.resize_counter_tab2) > 0.050:
            pass
            '''
            if self.t2c2i1.winfo_width() > 1:
                self.resize_counter_tab2 = time.time()
                self.master.update()
                self.t2c2i1_width = self.t2c2i1.winfo_width()
                self.t2c2i1_height = self.t2c2i1.winfo_height()
                self.crop_resized_im = self.resizeIm(self.crop_im,self.crop_width,self.crop_height,self.t2c2i1_width,self.t2c2i1_height)
                self.crop_tk = self.im2tk(self.crop_resized_im)
                self.t2c2i1.configure(image=self.crop_tk)
            '''

    def resizeIm(self,image,image_width,image_height,width_restrict,height_restrict):
        """
        Resizes PIL image according to given bounds
        @type  image: PIL image
        @param image: PIL image that you want to crop

        @type  image_width: integer
        @param image_width: the original image width in pixels

        @type  image_height: integer
        @param image_height: the original image height in pixels

        @type  width_restrict: integer
        @param width_restrict: the width in pixels of restricted area

        @type  height_restrict: integer
        @param height_restrict: the height in pixels of restricted area

        @rtype:  PIL image
        @return: Resized PIL image
        """
        ratio_h = height_restrict/image_height
        ratio_w = width_restrict/image_width
        if ratio_h <= ratio_w:
            resized_im = image.resize((int(image_width*ratio_h), int(image_height*ratio_h)), Image.ANTIALIAS)
        else:
            resized_im = image.resize((int(image_width*ratio_w), int(image_height*ratio_w)), Image.ANTIALIAS)
        return(resized_im)

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
        self.crop_preview_resized_im = self.resizeIm(self.crop_preview_im,self.crop_preview_width,self.crop_preview_height,self.t1c1i1_width*self.crop_preview_img_ratio,self.t1c1i1_height*self.crop_preview_img_ratio)
        self.crop_preview_tk = self.im2tk(self.crop_preview_resized_im)
        self.t1c2i1.configure(image=self.crop_preview_tk)

    def undoCrop(self,event=None):
        """
        Undoes crop and resets the raw image

        @type  event: event
        @param event: Ctrl + Z event

        @rtype:  None
        @return: None
        """
        self.draw_np = np.copy(self.org_np)
        self.img_im = self.np2im(self.draw_np)
        self.resized_im = self.resizeIm(self.img_im,self.org_width,self.org_height,self.t1c1i1_width,self.t1c1i1_height)
        self.img_tk = self.im2tk(self.resized_im)
        self.t1c1i1.configure(image=self.img_tk)
        self.crop_preview_im = self.img_im.copy()
        self.crop_preview_width,self.crop_preview_height = self.crop_preview_im.size
        self.crop_preview_resized_im = self.resizeIm(self.crop_preview_im,self.crop_preview_width,self.crop_preview_height,self.t1c1i1_width*self.crop_preview_img_ratio,self.t1c1i1_height*self.crop_preview_img_ratio)
        self.crop_preview_tk = self.im2tk(self.crop_preview_resized_im)
        self.t1c2i1.configure(image=self.crop_preview_tk)
        #self.t2c2i1.configure(image=self.crop_preview_tk)

    def nextRaw(self,event):
        """
        Requests and displays next raw image

        @type  event: event
        @param event: Right arrow event

        @rtype:  None
        @return: None
        """
        if self.serverConnected:
            time0 = time.time()
            query = self.interface.getNextRawImage(True)
            if query == None:
                self.noNextRaw()
            else:
                self.imageID = query[1]
                self.org_np = np.array(query[0]) #self.get_image('frame0744.jpg')
            time1 = time.time()
            self.draw_np = np.copy(self.org_np)
            self.img_im = self.np2im(self.draw_np)
            self.crop_preview_im = self.img_im.copy()
            self.org_width,self.org_height = self.img_im.size
            self.crop_preview_width,self.crop_preview_height = self.crop_preview_im.size
            self.cropped = False
            self.resized_im = self.resizeIm(self.img_im,self.org_width,self.org_height,self.t1c1i1_width,self.t1c1i1_height)
            self.img_tk = self.im2tk(self.resized_im)
            self.t1c1i1.configure(image=self.img_tk)
            self.crop_preview_resized_im = self.resizeIm(self.crop_preview_im,self.crop_preview_width,self.crop_preview_height,self.t1c1i1_width*self.crop_preview_img_ratio,self.t1c1i1_height*self.crop_preview_img_ratio)
            self.crop_preview_tk = self.im2tk(self.crop_preview_resized_im)
            self.t1c2i1.configure(image=self.crop_preview_tk)
            time2 = time.time()
            print("server request = ",time1-time0,"gui = ",time2-time1)

    def previousRaw(self,event):
        """
        Requests and displays previous raw image

        @type  event: event
        @param event: Left arrow event

        @rtype:  None
        @return: None
        """
        if self.serverConnected:
            time0 = time.time()
            query = self.interface.getPrevRawImage()
            if query == None:
                self.noPreviousRaw()
            else:
                self.imageID = query[1]
                self.org_np = np.array(query[0]) #self.get_image('frame0744.jpg')
            time1 = time.time()
            self.draw_np = np.copy(self.org_np)
            self.img_im = self.np2im(self.draw_np)
            self.crop_preview_im = self.img_im.copy()
            self.crop_preview_tk = self.im2tk(self.crop_preview_im)
            self.org_width,self.org_height = self.img_im.size
            self.crop_preview_width,self.crop_preview_height = self.img_im.size
            self.cropped = False
            self.resized_im = self.resizeIm(self.img_im,self.org_width,self.org_height,self.t1c1i1_width,self.t1c1i1_height)
            self.img_tk = self.im2tk(self.resized_im)
            self.t1c1i1.configure(image=self.img_tk)
            self.crop_preview_resized_im = self.resizeIm(self.crop_preview_im,self.crop_preview_width,self.crop_preview_height,self.t1c1i1_width*self.crop_preview_img_ratio,self.t1c1i1_height*self.crop_preview_img_ratio)
            self.crop_preview_tk = self.im2tk(self.crop_preview_resized_im)
            self.t1c2i1.configure(image=self.crop_preview_tk)
            time2 = time.time()
            print("server request = ",time1-time0,"gui = ",time2-time1)


    def submitCropped(self,event=None):
        """
        Submits cropped image to server

        @type  event: event
        @param event: Enter press or button press event

        @rtype:  None
        @return: None
        """
        self.interface.postCroppedImage(self.imageID,self.crop_preview_im,[self.cx0,self.cy0],[self.cx1,self.cy1])
        (self.cx0,self.cy0,self.cx1,self.cy1)


    def nextCropped(self,event):
        """
        Requests and displays next cropped image

        @type  event: event
        @param event: Right arrow event

        @rtype:  None
        @return: None
        """
        if self.serverConnected:
            time0 = time.time()
            query = self.interface.getNextCroppedImage()
            if query == None:
                self.noNextCropped()
            else:
                self.imageID = query[1]
                self.cropped_np = np.array(query[0]) #self.get_image('frame0744.jpg')
            time1 = time.time()
            self.cropped_im = self.np2im(self.cropped_np)
            self.cropped_width,self.cropped_height = self.cropped_im.size
            self.cropped_resized_im = self.resizeIm(self.cropped_im,self.cropped_width,self.cropped_height,self.t1c1i1_width,self.t1c1i1_height)
            self.cropped_tk = self.im2tk(self.cropped_resized_im)
            self.t2c2i1.configure(image=self.cropped_tk)
            time2 = time.time()
            print("server request = ",time1-time0,"gui = ",time2-time1)

    def previousCropped(self,event):
        """
        Requests and displays previous cropped image

        @type  event: event
        @param event: Left arrow event

        @rtype:  None
        @return: None
        """
        if self.serverConnected:
            time0 = time.time()
            query = self.interface.getPrevCroppedImage()
            if query == None:
                self.noPreviousCropped()
            else:
                self.imageID = query[1]
                self.cropped_np = np.array(query[0]) #self.get_image('frame0744.jpg')
            time1 = time.time()
            self.cropped_im = self.np2im(self.cropped_np)
            self.cropped_width,self.cropped_height = self.cropped_im.size
            self.cropped_resized_im = self.resizeIm(self.cropped_im,self.cropped_width,self.cropped_height,self.t1c1i1_width,self.t1c1i1_height)
            self.cropped_tk = self.im2tk(self.cropped_resized_im)
            self.t2c2i1.configure(image=self.cropped_tk)
            time2 = time.time()
            print("server request = ",time1-time0,"gui = ",time2-time1)




    def submitClassification(self,event=None):
        """
        Submits classification of image to server

        @type  event: event
        @param event: Enter press event

        @rtype:  None
        @return: None
        """
        shape = self.t2c2l2_var.get()
        alphanumeric = self.t2c2l4.get()
        orientation = self.t2c2l6_var.get()
        background_color = self.t2c2l10_var.get()
        alpha_color = self.t2c2l12_var.get()
        type = self.t2c2l14_var.get()
        description = self.t2c2l16.get()
        print(shape,alphanumeric,orientation)
        print(background_color,alpha_color,type,description)

    def tabChanged(self,event):
        """
        Performs the correct keybindings when you move to a new tab of the gui

        @type  event: event
        @param event: Tab changed event

        @rtype:  None
        @return: None
        """
        active_tab = self.n.index(self.n.select())
        if active_tab == 0:
            self.resizeEventTab0()
            self.master.unbind("<Right>")
            self.master.unbind("<Left>")
            self.master.unbind("<d>")
            self.master.unbind("<a>")
            self.master.bind("<Configure>",self.resizeEventTab0)
            self.master.unbind("<Control-z>")
            self.master.bind("<Return>",self.updateSettings)
            self.master.bind("<Escape>",self.close_window)
        if active_tab == 1:
            self.resizeEventTab1()
            self.master.bind("<Right>",self.nextRaw)
            self.master.bind("<Left>",self.previousRaw)
            self.master.unbind("<d>")
            self.master.unbind("<a>")
            self.master.bind("<Configure>",self.resizeEventTab1)
            self.master.bind("<Control-z>",self.undoCrop)
            self.master.bind("<Return>",self.submitCropped)
            self.master.bind("<Escape>",self.close_window)
        elif active_tab == 2:
            self.resizeEventTab2()
            self.master.bind("<Right>",self.nextCropped)
            self.master.bind("<Left>",self.previousCropped)
            self.master.unbind("<d>")
            self.master.unbind("<a>")
            self.master.bind("<Configure>",self.resizeEventTab2)
            self.master.unbind("<Control-z>")
            self.master.bind("<Return>",self.submitClassification)
            self.master.bind("<Escape>",self.close_window)
        elif active_tab == 3:
            self.master.unbind("<Right>")
            self.master.unbind("<Left>")
            self.master.unbind("<d>")
            self.master.unbind("<a>")
            self.master.unbind("<Configure>")
            self.master.unbind("<Control-z>")
            self.master.unbind("<Return>")
            self.master.bind("<Escape>",self.close_window)
        self.master.focus_set()

    def updateSettings(self,event=None):
        """
        Attempts to connect to server when settings are changed

        @type  event: event
        @param event: Enter press or button press event

        @rtype:  None
        @return: None
        """
        host_new = self.t0c2host.get()
        port_new = self.t0c2port.get()
        ids_new = int(self.t0c2ids.get())
        debug_new = not(self.t0c2debug.get())
        self.interface = client_rest.ImagingInterface(host=host_new,port=port_new,numIdsStored=ids_new,isDebug=debug_new)
        self.pingServer()
        self.draw_np = np.copy(self.org_np)
        self.img_im = self.np2im(self.draw_np)
        self.crop_preview_im = self.img_im.copy()
        self.org_width,self.org_height = self.img_im.size
        self.crop_preview_width,self.crop_preview_height = self.crop_preview_im.size
        if self.initialized and self.t1c1i1.winfo_width() > 1:
            self.resized_im = self.resizeIm(self.img_im,self.org_width,self.org_height,self.t1c1i1_width,self.t1c1i1_height)
            self.img_tk = self.im2tk(self.resized_im)
            self.crop_preview_resized_im = self.resizeIm(self.crop_preview_im,self.crop_preview_width,self.crop_preview_height,self.t1c1i1_width*self.crop_preview_img_ratio,self.t1c1i1_height*self.crop_preview_img_ratio)
            self.crop_preview_tk = self.im2tk(self.crop_preview_resized_im)
        else:
            self.img_tk = self.im2tk(self.img_im)
            self.crop_preview_tk = self.im2tk(self.crop_preview_im)
        self.t1c1i1.configure(image=self.img_tk)
        self.t1c2i1.configure(image=self.crop_preview_tk)

    def pingServer(self):
        """
        Checks if server is correctly connected

        @rtype:  None
        @return: None
        """
        self.serverConnected = self.interface.ping()
        if self.serverConnected:
            self.org_np = self.get_image('instructions.jpg')
            self.cropped_np = self.get_image('classify_instructions.jpg')
        else:
            self.org_np = self.get_image('server_error.jpg')
            self.cropped_np = self.get_image('server_error.jpg')

    def noNextRaw(self):
        """
        Checks if server is correctly connected

        @rtype:  None
        @return: None
        """
        self.serverConnected = self.interface.ping()
        if self.serverConnected:
            self.org_np = self.get_image('noNextRaw.jpg')
        else:
            self.org_np = self.get_image('server_error.jpg')

    def noPreviousRaw(self):
        """
        Checks if server is correctly connected

        @rtype:  None
        @return: None
        """
        self.serverConnected = self.interface.ping()
        if self.serverConnected:
            self.org_np = self.get_image('noPreviousRaw.jpg')
        else:
            self.org_np = self.get_image('server_error.jpg')

    def noNextCropped(self):
        """
        Checks if server is correctly connected

        @rtype:  None
        @return: None
        """
        self.serverConnected = self.interface.ping()
        if self.serverConnected:
            self.cropped_np = self.get_image('noNextCropped.jpg')
        else:
            self.cropped_np = self.get_image('server_error.jpg')

    def noPreviousCropped(self):
        """
        Checks if server is correctly connected

        @rtype:  None
        @return: None
        """
        self.serverConnected = self.interface.ping()
        if self.serverConnected:
            self.cropped_np = self.get_image('noPreviousCropped.jpg')
        else:
            self.cropped_np = self.get_image('server_error.jpg')

    def disableEmergentDescription(self,*args):
        """
        Disables emergent discription unless emergent target selected

        @rtype:  None
        @return: None
        """
        if self.t2c2l14_var.get() == 'emergent':
            self.t2c2l16.configure(state=tk.NORMAL)
        else:
            self.t2c2l16.configure(state=tk.DISABLED)








if __name__ == "__main__":
    root = tk.Tk()
    style = ThemedStyle(root)
    style.set_theme("arc")
    gui = GuiClass(root)
    try:
        gui.mainloop()
    except KeyboardInterrupt:
        root.destroy()
        sys.exit()
