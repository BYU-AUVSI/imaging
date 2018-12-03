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
    Submit Cropped
    Next Cropped
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
    Add preview of cropped image
    Add submit crop button
    Add sample pic of targets
    Add quantity of each target pictures
Tab2:
    Fix bug where misssing first value
    Disable description unless emergent
    Rotate image according to heading
    Add classified Targets
    Add classification queue
Tab3:
    Make everything

KNOWN BUGS:
    Weird exit is bound to an event (doesn't do anything)
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
    """Classification Gui for AUVSi 2019"""
    def __init__(self,master=None):
        tk.Frame.__init__(self,master=None)
        self.master = master # gui master handle
        try:
            self.master.attributes('-zoomed', True) # maximizes screen
        except (Exception) as e:
            w = Toplevel(root)
            w.state('zoomed')
        self.master.title("BYU AUVSI COMPETITION 2019")


        self.n = ttk.Notebook(self.master) # create tabs
        self.n.pack(fill=tk.BOTH, expand=1) # expand across space
        tk.Grid.rowconfigure(self.master,0,weight=1)
        tk.Grid.columnconfigure(self.master,0,weight=1)


        # itialize variables
        self.default_host = '192.168.1.48'
        #self.default_host = '127.0.0.1'
        self.default_port = '5000'
        self.default_idnum = 50
        self.default_debug = False
        self.interface = client_rest.ImagingInterface(host=self.default_host,port=self.default_port,numIdsStored=self.default_idnum,isDebug=self.default_debug)
        self.initialized = False
        self.target_number = 0
        self.org_np = np.array(self.interface.getNextRawImage(True))
        #self.org_np = self.get_image('frame0744.jpg')
        self.draw_np = np.copy(self.org_np)
        self.img_im = self.np2im(self.draw_np)
        self.crop_im = self.img_im.copy()
        self.crop_tk = self.im2tk(self.crop_im)
        self.img_tk = self.im2tk(self.img_im)
        self.org_width,self.org_height = self.img_im.size
        self.crop_width,self.crop_height = self.img_im.size
        self.resize_counter_tab0 = time.time()
        self.resize_counter_tab1 = time.time()
        self.resize_counter_tab2 = time.time()
        self.cropped = False

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
        if self.default_debug == False:
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
        #grid=Frame(self.tab1)
        #grid.grid(sticky=tk.N+tk.S+tk.E+tk.W,column=0,row=7,columnspan=2)
        #Grid.rowconfigure(self.tab1,0,weight=1)
        #Grid.columnconfigure(self.tab1,0,weight=1)

        for x in range(19):
            tk.Grid.columnconfigure(self.tab1,x,weight=1)
        for y in range(13):
            tk.Grid.rowconfigure(self.tab1,y,weight=1)



        self.lbl2 = ttk.Label(self.tab1, anchor=tk.CENTER, text=self.target_number)
        self.lbl2.grid(row=12,column=0,sticky=tk.N+tk.S+tk.E+tk.W,padx=5,pady=5,ipadx=5,ipady=5)
        but1 = ttk.Button(self.tab1, text="Advance Target",command=self.advanceTarget)
        but1.grid(row=12,column=1,sticky=tk.N+tk.S+tk.E+tk.W,padx=5,pady=5,ipadx=5,ipady=5)
        self.lbl3 = ttk.Label(self.tab1, anchor=tk.CENTER,image=self.img_tk)
        self.lbl3.image = self.img_tk
        self.lbl3.grid(row=0,column=0,rowspan=12,columnspan=12,sticky=tk.N+tk.S+tk.E+tk.W,padx=5,pady=5,ipadx=5,ipady=5)
        self.lbl3.bind("<Button-1>",self.mouse_click)
        self.lbl4 = ttk.Label(self.tab1, anchor=tk.CENTER,text="initial")
        self.lbl4.grid(row=12,column=2,sticky=tk.N+tk.S+tk.E+tk.W,padx=5,pady=5,ipadx=5,ipady=5)
        self.t1sep12 = ttk.Separator(self.tab1, orient=tk.VERTICAL)
        self.t1sep12.grid(row=0, column=13, rowspan=13,sticky=tk.N+tk.S+tk.E+tk.W, pady=5)
        self.lbl1 = ttk.Label(self.tab1, anchor=tk.CENTER, text='Target Number')
        self.lbl1.grid(row=0,column=13,sticky=tk.N+tk.S+tk.E+tk.W,padx=5,pady=5,ipadx=5,ipady=5)
        self.lbl5 = ttk.Label(self.tab1, anchor=tk.CENTER, text='Target Pic')
        self.lbl5.grid(row=0,column=14,sticky=tk.N+tk.S+tk.E+tk.W,padx=5,pady=5,ipadx=5,ipady=5)
        self.lbl6 = ttk.Label(self.tab1, anchor=tk.CENTER, text='Pic Quantity')
        self.lbl6.grid(row=0,column=15,sticky=tk.N+tk.S+tk.E+tk.W,padx=5,pady=5,ipadx=5,ipady=5)
        self.t1sep23 = ttk.Separator(self.tab1, orient=tk.VERTICAL)
        self.t1sep23.grid(row=0, column=16, rowspan=13,sticky=tk.N+tk.S+tk.E+tk.W, pady=5)
        self.lbl7 = ttk.Label(self.tab1, anchor=tk.CENTER, text='Target Number')
        self.lbl7.grid(row=0,column=16,sticky=tk.N+tk.S+tk.E+tk.W,padx=5,pady=5,ipadx=5,ipady=5)
        self.lbl8 = ttk.Label(self.tab1, anchor=tk.CENTER, text='Target Pic')
        self.lbl8.grid(row=0,column=17,sticky=tk.N+tk.S+tk.E+tk.W,padx=5,pady=5,ipadx=5,ipady=5)
        self.lbl9 = ttk.Label(self.tab1, anchor=tk.CENTER, text='Pic Quantity')
        self.lbl9.grid(row=0,column=18,sticky=tk.N+tk.S+tk.E+tk.W,padx=5,pady=5,ipadx=5,ipady=5)

        for ii in range(12):
            new_label1 = ttk.Label(self.tab1, anchor=tk.CENTER,text=ii+1)
            new_label1.grid(row=ii+1,column=13,sticky=tk.N+tk.S+tk.E+tk.W,padx=5,pady=5,ipadx=5,ipady=5)
            new_label1 = ttk.Label(self.tab1, anchor=tk.CENTER,text=ii+13)
            new_label1.grid(row=ii+1,column=16,sticky=tk.N+tk.S+tk.E+tk.W,padx=5,pady=5,ipadx=5,ipady=5)



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

        self.t2c2i1 = ttk.Label(self.tab2, anchor=tk.CENTER,image=self.crop_tk)
        self.t2c2i1.image = self.crop_tk
        self.t2c2i1.grid(row=2,column=4,rowspan=38,columnspan=8,sticky=tk.N+tk.S+tk.E+tk.W,padx=5,pady=5,ipadx=5,ipady=5)
        self.t2c2l1 = ttk.Label(self.tab2, anchor=tk.CENTER, text='Shape')
        self.t2c2l1.grid(row=40,column=4,columnspan=2,rowspan=2,sticky=tk.N+tk.S+tk.E+tk.W,padx=5,pady=5,ipadx=5,ipady=5)
        shape_options = ('circle','semicircle','quarter_circle','triangle','square','rectangle','trapezoid','pentagon','hexagon','heptagon','octagon','star','cross')
        self.t2c2l2_var = tk.StringVar(self.master)
        self.t2c2l2_var.set('circle')
        self.t2c2l2 = ttk.OptionMenu(self.tab2,self.t2c2l2_var,*shape_options)
        self.t2c2l2.grid(row=42,column=4,columnspan=2,rowspan=2,sticky=tk.N+tk.S+tk.E+tk.W,padx=5,pady=5,ipadx=5,ipady=5)
        self.t2c2l3 = ttk.Label(self.tab2, anchor=tk.CENTER, text='Alphanumeric')
        self.t2c2l3.grid(row=40,column=6,columnspan=2,rowspan=2,sticky=tk.N+tk.S+tk.E+tk.W,padx=5,pady=5,ipadx=5,ipady=5)
        self.t2c2l4 = ttk.Entry(self.tab2)
        self.t2c2l4.grid(row=42,column=6,columnspan=2,rowspan=2,sticky=tk.N+tk.S+tk.E+tk.W,padx=5,pady=5,ipadx=5,ipady=5)
        self.t2c2l5 = ttk.Label(self.tab2, anchor=tk.CENTER, text='Orientation')
        self.t2c2l5.grid(row=40,column=8,columnspan=4,rowspan=2,sticky=tk.N+tk.S+tk.E+tk.W,padx=5,pady=5,ipadx=5,ipady=5)
        orientation_options = ('N','NE','E','SE','S','SW','W','NW')
        self.t2c2l6_var = tk.StringVar(self.master)
        self.t2c2l6_var.set('N')
        self.t2c2l6 = ttk.OptionMenu(self.tab2,self.t2c2l6_var,*orientation_options)
        self.t2c2l6.grid(row=42,column=8,columnspan=4,rowspan=2,sticky=tk.N+tk.S+tk.E+tk.W,padx=5,pady=5,ipadx=5,ipady=5)
        self.t2c2l9 = ttk.Label(self.tab2, anchor=tk.CENTER, text='Background Color')
        self.t2c2l9.grid(row=44,column=4,columnspan=2,rowspan=2,sticky=tk.N+tk.S+tk.E+tk.W,padx=5,pady=5,ipadx=5,ipady=5)
        color_options = ('white','black','gray','red','blue','green','yellow','purple','brown','orange')
        self.t2c2l10_var = tk.StringVar(self.master)
        self.t2c2l10_var.set('white')
        self.t2c2l10 = ttk.OptionMenu(self.tab2,self.t2c2l10_var,*color_options)
        self.t2c2l10.grid(row=46,column=4,columnspan=2,rowspan=2,sticky=tk.N+tk.S+tk.E+tk.W,padx=5,pady=5,ipadx=5,ipady=5)
        self.t2c2l11 = ttk.Label(self.tab2, anchor=tk.CENTER, text='Alphanumeric Color')
        self.t2c2l11.grid(row=44,column=6,columnspan=2,rowspan=2,sticky=tk.N+tk.S+tk.E+tk.W,padx=5,pady=5,ipadx=5,ipady=5)
        self.t2c2l12_var = tk.StringVar(self.master)
        self.t2c2l12_var.set('white')
        self.t2c2l12 = ttk.OptionMenu(self.tab2,self.t2c2l12_var,*color_options)
        self.t2c2l12.grid(row=46,column=6,columnspan=2,rowspan=2,sticky=tk.N+tk.S+tk.E+tk.W,padx=5,pady=5,ipadx=5,ipady=5)
        self.t2c2l13 = ttk.Label(self.tab2, anchor=tk.CENTER, text='Target Type')
        self.t2c2l13.grid(row=44,column=8,columnspan=2,rowspan=2,sticky=tk.N+tk.S+tk.E+tk.W,padx=5,pady=5,ipadx=5,ipady=5)
        self.t2c2l14_var = tk.StringVar(self.master)
        target_options = ('standard','emergent','off-axis')
        self.t2c2l14_var.set('standard')
        self.t2c2l14 = ttk.OptionMenu(self.tab2,self.t2c2l14_var,*target_options)
        self.t2c2l14.grid(row=46,column=8,columnspan=2,rowspan=2,sticky=tk.N+tk.S+tk.E+tk.W,padx=5,pady=5,ipadx=5,ipady=5)
        self.t2c2l15 = ttk.Label(self.tab2, anchor=tk.CENTER, text='Emergent Description')
        self.t2c2l15.grid(row=44,column=10,columnspan=2,rowspan=2,sticky=tk.N+tk.S+tk.E+tk.W,padx=5,pady=5,ipadx=5,ipady=5)
        self.t2c2l16 = ttk.Entry(self.tab2)
        self.t2c2l16.grid(row=46,column=10,columnspan=2,rowspan=2,sticky=tk.N+tk.S+tk.E+tk.W,padx=5,pady=5,ipadx=5,ipady=5)
        self.t2c2l17 = ttk.Button(self.tab2, text="Submit Classification",command=self.submitClassification)
        self.t2c2l17.grid(row=48,column=4,columnspan=8,rowspan=2,sticky=tk.N+tk.S+tk.E+tk.W,padx=5,pady=5,ipadx=5,ipady=5)

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
        self.master.bind("<Escape>",self.close_window) # press ESC to exit


    def get_image(self,path):
        image_np = cv2.imread(path)
        image_np = cv2.cvtColor(image_np,cv2.COLOR_BGR2RGB)
        return image_np
    def np2im(self,image):
        image_im = Image.fromarray(image)
        return image_im
    def im2tk(self,image):
        image_tk = ImageTk.PhotoImage(image)
        return image_tk
    def bt1_clicked(self):
        self.target_number += 1
        self.lbl2.configure(text=self.target_number)
    def mouse_click(self,event):
        self.lbl4.configure(text=(event.x,event.y))
        self.lbl3.bind("<Motion>",self.mouse_move)
        self.offset_x = int((self.t1l1_width - self.t1i1_width)/2.0)
        self.offset_y = int((self.t1l1_height - self.t1i1_height)/2.0)
        self.x0 = event.x - self.offset_x
        self.y0 = event.y - self.offset_y
    def mouse_move(self,event):
        self.lbl4.configure(text=(event.x,event.y))
        self.lbl3.bind("<ButtonRelease-1>",self.mouse_release)
        self.x1 = event.x - self.offset_x
        self.y1 = event.y - self.offset_y
        disp_width,disp_height = self.resized_im.size
        sr = (self.org_width/disp_width + self.org_height/disp_height)/2.0
        self.draw_np = np.copy(self.org_np)
        cv2.rectangle(self.draw_np,(int(sr*self.x0),int(sr*self.y0)),(int(sr*self.x1),int(sr*self.y1)),(255,0,0),2)
        self.img_im = self.np2im(self.draw_np)
        self.resized_im = self.resizeIm(self.img_im,self.org_width,self.org_height,self.t1l1_width,self.t1l1_height)
        self.img_tk = self.im2tk(self.resized_im)
        self.lbl3.configure(image=self.img_tk)
    def close_window(self,event):
        self.master.destroy()
        sys.exit()
    def mouse_release(self,event):
        if self.cropped == True:
            self.undoCrop()
        self.lbl3.unbind("<Motion>")
        self.lbl4.configure(text=(event.x,event.y))
        self.lbl3.unbind("<ButtonRelease-1>")
        self.x1 = event.x - self.offset_x
        self.y1 = event.y - self.offset_y
        disp_width,disp_height = self.resized_im.size
        sr = (self.org_width/disp_width + self.org_height/disp_height)/2.0
        self.draw_np = np.copy(self.org_np)
        cv2.rectangle(self.draw_np,(int(sr*self.x0),int(sr*self.y0)),(int(sr*self.x1),int(sr*self.y1)),(255,0,0),2)
        self.img_im = self.np2im(self.draw_np)
        self.resized_im = self.resizeIm(self.img_im,self.org_width,self.org_height,self.t1l1_width,self.t1l1_height)
        self.img_tk = self.im2tk(self.resized_im)
        self.lbl3.configure(image=self.img_tk)
        # Crop Image
        self.cropImage(int(sr*self.x0),int(sr*self.y0),int(sr*self.x1),int(sr*self.y1))
        self.cropped = True
    def resizeEventTab0(self,event=None):
        if self.initialized == True and (time.time()-self.resize_counter_tab0) > 0.050:
            if self.t0c2i1.winfo_width() > 1:
                self.resize_counter_tab0 = time.time()
                self.master.update()
                self.t0c2i1_width = self.t0c2i1.winfo_width()
                self.t0c2i1_height = self.t0c2i1.winfo_height()
                self.logo_resized_im = self.resizeIm(self.logo_im,self.logo_width,self.logo_height,self.t0c2i1_width,self.t0c2i1_height)
                self.t0c2i1_width,self.t0c2i1_height = self.logo_resized_im.size
                self.logo_tk = self.im2tk(self.logo_resized_im)
                self.t0c2i1.configure(image=self.logo_tk)
    def resizeEventTab1(self,event=None):
        if self.initialized == True and (time.time()-self.resize_counter_tab1) > 0.050:
            if self.lbl3.winfo_width() > 1:
                self.resize_counter_tab1 = time.time()
                self.master.update()
                self.t1l1_width = self.lbl3.winfo_width()
                self.t1l1_height = self.lbl3.winfo_height()
                self.resized_im = self.resizeIm(self.img_im,self.org_width,self.org_height,self.t1l1_width,self.t1l1_height)
                self.t1i1_width,self.t1i1_height = self.resized_im.size
                self.img_tk = self.im2tk(self.resized_im)
                self.lbl3.configure(image=self.img_tk)
    def resizeEventTab2(self,event=None):
        if self.initialized == True and (time.time()-self.resize_counter_tab2) > 0.050:
            if self.t2c2i1.winfo_width() > 1:
                self.resize_counter_tab2 = time.time()
                self.master.update()
                self.t2c2i1_width = self.t2c2i1.winfo_width()
                self.t2c2i1_height = self.t2c2i1.winfo_height()
                self.crop_resized_im = self.resizeIm(self.crop_im,self.crop_width,self.crop_height,self.t2c2i1_width,self.t2c2i1_height)
                self.crop_tk = self.im2tk(self.crop_resized_im)
                self.t2c2i1.configure(image=self.crop_tk)
    def resizeIm(self,image,image_width,image_height,width_restrict,height_restrict):
        ratio_h = height_restrict/image_height
        ratio_w = width_restrict/image_width
        if ratio_h <= ratio_w:
            resized_im = image.resize((int(image_width*ratio_h), int(image_height*ratio_h)), Image.ANTIALIAS)
        else:
            resized_im = image.resize((int(image_width*ratio_w), int(image_height*ratio_w)), Image.ANTIALIAS)
        return(resized_im)
    def cropImage(self,x0,y0,x1,y1):
        if x0 < x1:
            cx0 = x0
            cx1 = x1
        else:
            cx0 = x1
            cx1 = x0
        if y0 < y1:
            cy0 = y0
            cy1 = y1
        else:
            cy0 = y1
            cy1 = y0
        self.crop_im = self.crop_im.crop((cx0,cy0,cx1,cy1))
        self.crop_width,self.crop_height = self.crop_im.size
        self.crop_tk = self.im2tk(self.crop_im)
        self.t2c2i1.configure(image=self.crop_tk)
    def undoCrop(self,event=None):
        self.draw_np = np.copy(self.org_np)
        self.img_im = self.np2im(self.draw_np)
        self.resized_im = self.resizeIm(self.img_im,self.org_width,self.org_height,self.t1l1_width,self.t1l1_height)
        self.img_tk = self.im2tk(self.resized_im)
        self.lbl3.configure(image=self.img_tk)
        self.crop_im = self.img_im.copy()
        self.crop_width,self.crop_height = self.crop_im.size
        self.crop_tk = self.im2tk(self.crop_im)
        self.t2c2i1.configure(image=self.crop_tk)
    def advanceTarget(self,event=None):
        self.target_number += 1
        self.lbl2.configure(text=self.target_number)
    def decrementTarget(self,event):
        self.target_number -= 1
        if self.target_number < 0:
            self.target_number = 0
        self.lbl2.configure(text=self.target_number)
    def nextRaw(self,event):
        time0 = time.time()
        self.org_np = np.array(self.interface.getNextRawImage(True)) #self.get_image('frame0744.jpg')
        time1 = time.time()
        self.draw_np = np.copy(self.org_np)
        self.img_im = self.np2im(self.draw_np)
        self.crop_im = self.img_im.copy()
        self.crop_tk = self.im2tk(self.crop_im)
        self.org_width,self.org_height = self.img_im.size
        self.crop_width,self.crop_height = self.img_im.size
        self.cropped = False
        self.resized_im = self.resizeIm(self.img_im,self.org_width,self.org_height,self.t1l1_width,self.t1l1_height)
        self.t1i1_width,self.t1i1_height = self.resized_im.size
        self.img_tk = self.im2tk(self.resized_im)
        self.lbl3.configure(image=self.img_tk)
        self.t2c2i1.configure(image=self.crop_tk)
        time2 = time.time()
        print("server request = ",time1-time0,"gui = ",time2-time1)
    def previousRaw(self,event):
        print("previous Raw")
    def nextCropped(self,event):
        print("next Cropped")
    def submitCropped(self,event=None):
        print("submit Crop")
    def submitClassification(self,event=None):
        shape = self.t2c2l2_var.get()
        alphanumeric = self.t2c2l4.get()
        orientation = self.t2c2l6_var.get()
        background_color = self.t2c2l10_var.get()
        alpha_color = self.t2c2l12_var.get()
        type = self.t2c2l14_var.get()
        description = self.t2c2l16.get()
        print("submit classification")
        print(shape,alphanumeric,orientation)
        print(background_color,alpha_color,type,description)
    def tabChanged(self,event):
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
        if active_tab == 1:
            self.resizeEventTab1()
            self.master.bind("<Right>",self.nextRaw)
            self.master.bind("<Left>",self.previousRaw)
            self.master.bind("<d>",self.advanceTarget)
            self.master.bind("<a>",self.decrementTarget)
            self.master.bind("<Configure>",self.resizeEventTab1)
            self.master.bind("<Control-z>",self.undoCrop)
            self.master.bind("<Return>",self.submitCropped)
        elif active_tab == 2:
            self.resizeEventTab2()
            self.master.unbind("<Right>")
            self.master.unbind("<Left>")
            self.master.unbind("<d>")
            self.master.unbind("<a>")
            self.master.bind("<Configure>",self.resizeEventTab2)
            self.master.unbind("<Control-z>")
            self.master.bind("<Return>",self.submitClassification)
        elif active_tab == 3:
            self.master.unbind("<Right>")
            self.master.unbind("<Left>")
            self.master.unbind("<d>")
            self.master.unbind("<a>")
            self.master.unbind("<Configure>")
            self.master.unbind("<Control-z>")
            self.master.unbind("<Return>")
        self.master.focus_set()
    def updateSettings(self,event=None):
        host_new = self.t0c2host.get()
        port_new = self.t0c2port.get()
        ids_new = int(self.t0c2ids.get())
        debug_new = not(self.t0c2debug.get())
        self.interface = client_rest.ImagingInterface(host=host_new,port=port_new,numIdsStored=ids_new,isDebug=debug_new)








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
