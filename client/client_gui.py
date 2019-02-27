'''
Authors: D. Knowles, B. McBride, T. Miller
'''

'''
TODO:

All:
    possible threading behind the scenese to autosize other tabs
    use consistent naming patterns (camel case, snake case)
    use consistent commenting pattern and placement
    change gui into multiple tabs/classes
    change text font, size, color, etc.
    organize functions into a logical order
    standardize import order
Tab0:
    add error handling if entries aren't in the right format
    add error handling if not connected to correct wifi
Tab1:
    Add zooming feature
    Fix resizing
Tab2
    show not only submission status, but what you submitted??
    Change disable color
    Verify rotating picture based on yaw angle
    Show past classifications on the left with autofill option
    Bug tabbing into emergent description and trying to leave w/o subbmitting
Tab3:
    Show in blue which target it's pulling the "to submit" classificaiton from
    Change radiobuttons to match ttktheme
    Fix resizing issues
    Possibly auto updating? (if other people are pushing classifications)
Tab4:
    create everything

KNOWN BUGS:
    When you click on the 2nd tab right at the beginning, and then use the left/right
        buttons, it moves one tab, then unbinds like it's supposed to.
'''

import tkinter as tk
from tkinter import ttk, font
from ttkthemes import ThemedStyle
from PIL import Image,ImageTk
import PIL
import cv2
import numpy as np
import time
import sys
import imutils
from lib import tab0, tab1, tab_tools, client_rest


class GuiClass(tk.Frame):
    """
    Graphical User Interface for 2019 AUVSI competition
    Tab 2: Pull cropped iamges and submit classification for images
    Tab 3: Display results for manual classification

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
        self.master.bind("<Escape>",self.close_window)

        # -----------------------  Tab 0: SETTINGS  ----------------------------
        self.tab0 = tab0.Tab0(self.master,self.n)
        self.interface = self.tab0.interfaceCall()

        # -----------------------  Tab 1: SETTINGS  ----------------------------
        self.tab1 = tab1.Tab1(self.master,self.n,self.interface)


        self.pingServerTab2()
        self.cropped_im = tab_tools.np2im(self.cropped_np)
        self.cropped_width,self.cropped_height = self.cropped_im.size
        self.cropped_tk = tab_tools.im2tk(self.cropped_im)


        self.resize_counter_tab2 = time.time()
        self.resize_counter_tab3 = time.time()


        # Tab 2 variables
        self.t2_functional = False # prevent
        self.t2_entry_focus = False
        # Tab 3 variables
        self.t3_total_targets  = 0
        self.t3_current_target = 1
        self.submit_crop_id = None
        self.submit_orientation = None
        self.submit_bg_color = None
        self.submit_alpha_color = None
        self.submit_desc = None


        '''
        # new radiobutton Style
        s = ttk.Style()
        s.configure('Centered.TRadiobutton',width=30,justify=tk.CENTER)
        s.configure('Centered.TRadiobutton',justify=tk.CENTER)
        '''




        # TAB 2: CLASSIFICATION ------------------------------------------------
        self.tab2 = ttk.Frame(self.n)   # second page
        self.n.add(self.tab2, text='Classification')

        for x in range(16):
            tk.Grid.columnconfigure(self.tab2,x,weight=1)
        for y in range(50):
            tk.Grid.rowconfigure(self.tab2,y,weight=1)

        # Column One
        self.t2c1title = ttk.Label(self.tab2, anchor=tk.CENTER, text='                  ')
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
        self.t2c2l2 = ttk.OptionMenu(self.tab2,self.t2c2l2_var,shape_options[0],*shape_options)
        self.t2c2l2.grid(row=42,column=4,columnspan=2,rowspan=2,sticky=tk.N+tk.S+tk.E+tk.W,padx=5,pady=5,ipadx=5,ipady=5)
        self.t2c2l3 = ttk.Label(self.tab2, anchor=tk.CENTER, text='Alphanumeric')
        self.t2c2l3.grid(row=40,column=6,columnspan=2,rowspan=2,sticky=tk.N+tk.S+tk.E+tk.W,padx=5,pady=5,ipadx=5,ipady=5)
        self.t2c2l4_var = tk.StringVar(self.master)
        alphanumericValidateCommand = self.register(self.alphanumericValidate)
        self.t2c2l4 = ttk.Entry(self.tab2,textvariable=self.t2c2l4_var,validate=tk.ALL,validatecommand=(alphanumericValidateCommand, '%d','%P'))
        self.t2c2l4.grid(row=42,column=6,columnspan=2,rowspan=2,sticky=tk.N+tk.S+tk.E+tk.W,padx=5,pady=5,ipadx=5,ipady=5)
        self.t2c2l5 = ttk.Label(self.tab2, anchor=tk.CENTER, text='Orientation')
        self.t2c2l5.grid(row=40,column=8,columnspan=4,rowspan=2,sticky=tk.N+tk.S+tk.E+tk.W,padx=5,pady=5,ipadx=5,ipady=5)
        orientation_options = ('N','NE','E','SE','S','SW','W','NW')
        self.t2c2l6_var = tk.StringVar(self.master)
        self.t2c2l6 = ttk.OptionMenu(self.tab2,self.t2c2l6_var,orientation_options[0],*orientation_options)
        self.t2c2l6.grid(row=42,column=8,columnspan=4,rowspan=2,sticky=tk.N+tk.S+tk.E+tk.W,padx=5,pady=5,ipadx=5,ipady=5)
        self.t2c2l9 = ttk.Label(self.tab2, anchor=tk.CENTER, text='Background Color')
        self.t2c2l9.grid(row=44,column=4,columnspan=2,rowspan=2,sticky=tk.N+tk.S+tk.E+tk.W,padx=5,pady=5,ipadx=5,ipady=5)
        color_options = ('white','black','gray','red','blue','green','yellow','purple','brown','orange')
        self.t2c2l10_var = tk.StringVar(self.master)
        self.t2c2l10 = ttk.OptionMenu(self.tab2,self.t2c2l10_var,color_options[0],*color_options)
        self.t2c2l10.grid(row=46,column=4,columnspan=2,rowspan=2,sticky=tk.N+tk.S+tk.E+tk.W,padx=5,pady=5,ipadx=5,ipady=5)
        self.t2c2l11 = ttk.Label(self.tab2, anchor=tk.CENTER, text='Alphanumeric Color')
        self.t2c2l11.grid(row=44,column=6,columnspan=2,rowspan=2,sticky=tk.N+tk.S+tk.E+tk.W,padx=5,pady=5,ipadx=5,ipady=5)
        self.t2c2l12_var = tk.StringVar(self.master)
        self.t2c2l12 = ttk.OptionMenu(self.tab2,self.t2c2l12_var,color_options[0],*color_options)
        self.t2c2l12.grid(row=46,column=6,columnspan=2,rowspan=2,sticky=tk.N+tk.S+tk.E+tk.W,padx=5,pady=5,ipadx=5,ipady=5)
        self.t2c2l13 = ttk.Label(self.tab2, anchor=tk.CENTER, text='Target Type')
        self.t2c2l13.grid(row=44,column=8,columnspan=2,rowspan=2,sticky=tk.N+tk.S+tk.E+tk.W,padx=5,pady=5,ipadx=5,ipady=5)
        self.t2c2l14_var = tk.StringVar(self.master)
        target_options = ('standard','emergent','off_axis')
        self.t2c2l14 = ttk.OptionMenu(self.tab2,self.t2c2l14_var,target_options[0],*target_options)
        self.t2c2l14.grid(row=46,column=8,columnspan=2,rowspan=2,sticky=tk.N+tk.S+tk.E+tk.W,padx=5,pady=5,ipadx=5,ipady=5)
        self.t2c2l14_var.trace("w",self.disableEmergentDescription)

        self.t2c2l15 = ttk.Label(self.tab2, anchor=tk.CENTER, text='Emergent Description')
        self.t2c2l15.grid(row=44,column=10,columnspan=2,rowspan=2,sticky=tk.N+tk.S+tk.E+tk.W,padx=5,pady=5,ipadx=5,ipady=5)
        self.t2c2l16_var = tk.StringVar()
        self.t2c2l16_var.set(None)
        self.t2c2l16 = ttk.Entry(self.tab2,textvariable=self.t2c2l16_var)
        self.t2c2l16.grid(row=46,column=10,columnspan=2,rowspan=2,sticky=tk.N+tk.S+tk.E+tk.W,padx=5,pady=5,ipadx=5,ipady=5)

        self.t2c2r48a = ttk.Label(self.tab2, anchor=tk.E, text='Submission Status: ')
        self.t2c2r48a.grid(row=48,column=4,columnspan=2,rowspan=2,sticky=tk.N+tk.S+tk.E+tk.W,padx=5,pady=5,ipadx=5,ipady=5)
        self.t2c2lr48b = ttk.Label(self.tab2, anchor=tk.W, text='N/A')
        self.t2c2lr48b.grid(row=48,column=6,columnspan=2,rowspan=2,sticky=tk.N+tk.S+tk.E+tk.W,padx=5,pady=5,ipadx=5,ipady=5)

        self.t2c2l17 = ttk.Button(self.tab2, text="Submit Classification",command=self.submitClassification)
        self.t2c2l17.grid(row=48,column=8,columnspan=4,rowspan=2,sticky=tk.N+tk.S+tk.E+tk.W,padx=5,pady=5,ipadx=5,ipady=5)
        self.disableEmergentDescription()

        # Column Three
        self.t2sep23 = ttk.Separator(self.tab2, orient=tk.VERTICAL)
        self.t2sep23.grid(row=0, column=12, rowspan=50,sticky=tk.N+tk.S+tk.E+tk.W, pady=5)
        self.t2c3title = ttk.Label(self.tab2, anchor=tk.CENTER, text='                      ')
        self.t2c3title.grid(row=0,column=12,columnspan=4,sticky=tk.N+tk.S+tk.E+tk.W,padx=5,pady=5,ipadx=5,ipady=5)
        t2c2i1_np = tab_tools.get_image('assets/compass.jpg')
        self.t2c3i1_im = tab_tools.np2im(t2c2i1_np)
        self.t2c3i1_default_width,self.t2c3i1_default_height = self.t2c3i1_im.size
        self.t2c3i1_tk = tab_tools.im2tk(self.t2c3i1_im)
        # place image
        self.t2c3i1 = ttk.Label(self.tab2, anchor=tk.CENTER,image=self.t2c3i1_tk)
        self.t2c3i1.image = self.t2c3i1_tk
        self.t2c3i1.grid(row=2,column=12,rowspan=38,columnspan=4,sticky=tk.N+tk.S+tk.E+tk.W,padx=5,pady=5,ipadx=5,ipady=5)




        # TAB 3: MANUAL TARGET SUBMISSION ------------------------------------------------
        self.tab3 = ttk.Frame(self.n)
        self.n.add(self.tab3, text='Manual Target Submission')

        for x in range(12):
            tk.Grid.columnconfigure(self.tab3,x,weight=1)
        for y in range(21):
            tk.Grid.rowconfigure(self.tab3,y,weight=1)

        self.t3_default_np = tab_tools.get_image('assets/noClassifiedTargets.jpg')
        self.t3_default_im = tab_tools.np2im(self.t3_default_np)
        self.t3_default_width,self.t3_default_height = self.t3_default_im.size
        self.t3_default_tk = tab_tools.im2tk(self.t3_default_im)

        # Title
        self.t3titleA = ttk.Label(self.tab3, anchor=tk.CENTER, text='Target #')
        self.t3titleA.grid(row=0,column=4,columnspan=1,sticky=tk.N+tk.S+tk.E+tk.W,padx=5,pady=5,ipadx=5,ipady=5)
        self.t3titleB = ttk.Label(self.tab3, anchor=tk.CENTER, text=self.t3_current_target)
        self.t3titleB.grid(row=0,column=5,columnspan=1,sticky=tk.N+tk.S+tk.E+tk.W,padx=5,pady=5,ipadx=5,ipady=5)
        self.t3titleC = ttk.Label(self.tab3, anchor=tk.CENTER, text='Out of')
        self.t3titleC.grid(row=0,column=6,columnspan=1,sticky=tk.N+tk.S+tk.E+tk.W,padx=5,pady=5,ipadx=5,ipady=5)
        self.t3titleD = ttk.Label(self.tab3, anchor=tk.CENTER, text=self.t3_total_targets)
        self.t3titleD.grid(row=0,column=7,columnspan=1,sticky=tk.N+tk.S+tk.E+tk.W,padx=5,pady=5,ipadx=5,ipady=5)

        # Column One
        self.t3c1title = ttk.Label(self.tab3, anchor=tk.CENTER, text='Pic 1')
        self.t3c1title.grid(row=1,column=0,columnspan=2,sticky=tk.N+tk.S+tk.E+tk.W,padx=5,pady=5,ipadx=5,ipady=5)
        self.t3c1i1_im = self.t3_default_im.copy()
        self.t3c1i1_tk = tab_tools.im2tk(self.t3c1i1_im)
        self.t3c1i1_org_width,self.t3c1i1_org_height = self.t3c1i1_im.size
        self.t3c1i1 = ttk.Label(self.tab3, anchor=tk.CENTER,image=self.t3c1i1_tk)
        self.t3c1i1.image = self.t3c1i1_tk
        self.t3c1i1.grid(row=3,column=0,rowspan=1,columnspan=2,sticky=tk.N+tk.S+tk.E+tk.W,padx=5,pady=5,ipadx=5,ipady=5)
        # Characteristics
        self.submissionImage = tk.IntVar()
        self.t3c1r4 = tk.Radiobutton(self.tab3,value=1,variable=self.submissionImage,command=self.selectImage)
        self.t3c1r4.configure(foreground="#5c616c",background="#f5f6f7",highlightthickness=0,anchor=tk.N)
        self.t3c1r4.grid(row=4,column=0,columnspan=2,sticky=tk.N+tk.E+tk.W+tk.S,padx=5,pady=5,ipadx=5,ipady=5)
        self.t3c1ar5 = ttk.Label(self.tab3, anchor=tk.CENTER, text="Shape:")
        self.t3c1ar5.grid(row=5,column=0,columnspan=1,sticky=tk.N+tk.S+tk.E+tk.W,padx=5,pady=5,ipadx=5,ipady=5)
        self.t3c1br5 = ttk.Label(self.tab3, anchor=tk.CENTER, text="N/A")
        self.t3c1br5.grid(row=5,column=1,columnspan=1,sticky=tk.N+tk.S+tk.E+tk.W,padx=5,pady=5,ipadx=5,ipady=5)
        self.t3c1ar7 = ttk.Label(self.tab3, anchor=tk.S, text="Background Color:")
        self.t3c1ar7.grid(row=7,column=0,columnspan=1,sticky=tk.N+tk.S+tk.E+tk.W,padx=5,pady=5,ipadx=5,ipady=5)
        self.t3c1br7 = ttk.Label(self.tab3, anchor=tk.S, text="N/A")
        self.t3c1br7.grid(row=7,column=1,columnspan=1,sticky=tk.N+tk.S+tk.E+tk.W,padx=5,pady=5,ipadx=5,ipady=5)
        self.submissionBackgroundColor = tk.IntVar()
        self.t3c1r8 = tk.Radiobutton(self.tab3,value=1,variable=self.submissionBackgroundColor,command=self.selectBackgroundColor)
        self.t3c1r8.configure(foreground="#5c616c",background="#f5f6f7",highlightthickness=0,anchor=tk.N)
        self.t3c1r8.grid(row=8,column=0,columnspan=2,sticky=tk.N+tk.E+tk.W+tk.S,padx=5,pady=5,ipadx=5,ipady=5)
        self.t3c1ar9 = ttk.Label(self.tab3, anchor=tk.CENTER, text="Alphanumeric:")
        self.t3c1ar9.grid(row=9,column=0,columnspan=1,sticky=tk.N+tk.S+tk.E+tk.W,padx=5,pady=5,ipadx=5,ipady=5)
        self.t3c1br9 = ttk.Label(self.tab3, anchor=tk.CENTER, text="N/A")
        self.t3c1br9.grid(row=9,column=1,columnspan=1,sticky=tk.N+tk.S+tk.E+tk.W,padx=5,pady=5,ipadx=5,ipady=5)
        self.t3c1ar11 = ttk.Label(self.tab3, anchor=tk.S, text="Alphanumeric Color:")
        self.t3c1ar11.grid(row=11,column=0,columnspan=1,sticky=tk.N+tk.S+tk.E+tk.W,padx=5,pady=5,ipadx=5,ipady=5)
        self.t3c1br11 = ttk.Label(self.tab3, anchor=tk.S, text="N/A")
        self.t3c1br11.grid(row=11,column=1,columnspan=1,sticky=tk.N+tk.S+tk.E+tk.W,padx=5,pady=5,ipadx=5,ipady=5)
        self.submissionAlphanumericColor = tk.IntVar()
        self.t3c1r12 = tk.Radiobutton(self.tab3,value=1,variable=self.submissionAlphanumericColor,command=self.selectAlphanumericColor)
        self.t3c1r12.configure(foreground="#5c616c",background="#f5f6f7",highlightthickness=0,anchor=tk.N)
        self.t3c1r12.grid(row=12,column=0,columnspan=2,sticky=tk.N+tk.E+tk.W+tk.S,padx=5,pady=5,ipadx=5,ipady=5)
        self.t3c1ar13 = ttk.Label(self.tab3, anchor=tk.S, text="Orientation:")
        self.t3c1ar13.grid(row=13,column=0,columnspan=1,sticky=tk.N+tk.S+tk.E+tk.W,padx=5,pady=5,ipadx=5,ipady=5)
        self.t3c1br13 = ttk.Label(self.tab3, anchor=tk.S, text="N/A")
        self.t3c1br13.grid(row=13,column=1,columnspan=1,sticky=tk.N+tk.S+tk.E+tk.W,padx=5,pady=5,ipadx=5,ipady=5)
        self.submissionOrientation = tk.IntVar()
        self.t3c1r14 = tk.Radiobutton(self.tab3,value=1,variable=self.submissionOrientation,command=self.selectOrientation)
        self.t3c1r14.configure(foreground="#5c616c",background="#f5f6f7",highlightthickness=0,anchor=tk.N)
        self.t3c1r14.grid(row=14,column=0,columnspan=2,sticky=tk.N+tk.E+tk.W+tk.S,padx=5,pady=5,ipadx=5,ipady=5)
        self.t3c1ar15 = ttk.Label(self.tab3, anchor=tk.CENTER, text="Target Type:")
        self.t3c1ar15.grid(row=15,column=0,columnspan=1,sticky=tk.N+tk.S+tk.E+tk.W,padx=5,pady=5,ipadx=5,ipady=5)
        self.t3c1br15 = ttk.Label(self.tab3, anchor=tk.CENTER, text="N/A")
        self.t3c1br15.grid(row=15,column=1,columnspan=1,sticky=tk.N+tk.S+tk.E+tk.W,padx=5,pady=5,ipadx=5,ipady=5)
        self.t3c1r17 = ttk.Label(self.tab3, anchor=tk.CENTER, text="Description:")
        self.t3c1r17.grid(row=17,column=0,columnspan=2,sticky=tk.N+tk.S+tk.E+tk.W,padx=5,pady=5,ipadx=5,ipady=5)
        self.t3c1r18 = ttk.Label(self.tab3, anchor=tk.CENTER, text="N/A")
        self.t3c1r18.grid(row=18,column=0,columnspan=2,sticky=tk.N+tk.S+tk.E+tk.W,padx=5,pady=5,ipadx=5,ipady=5)
        self.submissionDescription = tk.IntVar()
        self.t3c1r19 = tk.Radiobutton(self.tab3,value=1,variable=self.submissionDescription,command=self.selectDescription)
        self.t3c1r19.configure(foreground="#5c616c",background="#f5f6f7",highlightthickness=0,anchor=tk.N)
        self.t3c1r19.grid(row=19,column=0,columnspan=2,sticky=tk.N+tk.E+tk.W+tk.S,padx=5,pady=5,ipadx=5,ipady=5)
        self.t3c1b1 = ttk.Button(self.tab3, text="Delete Classification",command=self.deleteClassification1)
        self.t3c1b1.grid(row=20,column=0,columnspan=2,sticky=tk.N+tk.S+tk.E+tk.W,padx=5,pady=5,ipadx=5,ipady=5)



        # Column Two
        self.t3c2title = ttk.Label(self.tab3, anchor=tk.CENTER, text='Pic 2')
        self.t3c2title.grid(row=1,column=2,columnspan=2,sticky=tk.N+tk.S+tk.E+tk.W,padx=5,pady=5,ipadx=5,ipady=5)
        self.t3c2i1_im = self.t3_default_im.copy()
        self.t3c2i1_tk = tab_tools.im2tk(self.t3c2i1_im)
        self.t3c2i1_org_width,self.t3c2i1_org_height = self.t3c2i1_im.size
        self.t3c2i1 = ttk.Label(self.tab3, anchor=tk.CENTER,image=self.t3c2i1_tk)
        self.t3c2i1.image = self.t3c2i1_tk
        self.t3c2i1.grid(row=3,column=2,rowspan=1,columnspan=2,sticky=tk.N+tk.S+tk.E+tk.W,padx=5,pady=5,ipadx=5,ipady=5)
        # Characteristics
        self.t3c2r4 = tk.Radiobutton(self.tab3,value=2,variable=self.submissionImage,command=self.selectImage)
        self.t3c2r4.configure(foreground="#5c616c",background="#f5f6f7",highlightthickness=0,anchor=tk.N)
        self.t3c2r4.grid(row=4,column=2,columnspan=2,sticky=tk.N+tk.E+tk.W+tk.S,padx=5,pady=5,ipadx=5,ipady=5)
        self.t3c2ar5 = ttk.Label(self.tab3, anchor=tk.CENTER, text="Shape:")
        self.t3c2ar5.grid(row=5,column=2,columnspan=1,sticky=tk.N+tk.S+tk.E+tk.W,padx=5,pady=5,ipadx=5,ipady=5)
        self.t3c2br5 = ttk.Label(self.tab3, anchor=tk.CENTER, text="N/A")
        self.t3c2br5.grid(row=5,column=3,columnspan=1,sticky=tk.N+tk.S+tk.E+tk.W,padx=5,pady=5,ipadx=5,ipady=5)
        self.t3c2ar7 = ttk.Label(self.tab3, anchor=tk.S, text="Background Color:")
        self.t3c2ar7.grid(row=7,column=2,columnspan=1,sticky=tk.N+tk.S+tk.E+tk.W,padx=5,pady=5,ipadx=5,ipady=5)
        self.t3c2br7 = ttk.Label(self.tab3, anchor=tk.S, text="N/A")
        self.t3c2br7.grid(row=7,column=3,columnspan=1,sticky=tk.N+tk.S+tk.E+tk.W,padx=5,pady=5,ipadx=5,ipady=5)
        self.t3c2r8 = tk.Radiobutton(self.tab3,value=2,variable=self.submissionBackgroundColor,command=self.selectBackgroundColor)
        self.t3c2r8.configure(foreground="#5c616c",background="#f5f6f7",highlightthickness=0,anchor=tk.N)
        self.t3c2r8.grid(row=8,column=2,columnspan=2,sticky=tk.N+tk.E+tk.W+tk.S,padx=5,pady=5,ipadx=5,ipady=5)
        self.t3c2ar9 = ttk.Label(self.tab3, anchor=tk.CENTER, text="Alphanumeric:")
        self.t3c2ar9.grid(row=9,column=2,columnspan=1,sticky=tk.N+tk.S+tk.E+tk.W,padx=5,pady=5,ipadx=5,ipady=5)
        self.t3c2br9 = ttk.Label(self.tab3, anchor=tk.CENTER, text="N/A")
        self.t3c2br9.grid(row=9,column=3,columnspan=1,sticky=tk.N+tk.S+tk.E+tk.W,padx=5,pady=5,ipadx=5,ipady=5)
        self.t3c2ar11 = ttk.Label(self.tab3, anchor=tk.S, text="Alphanumeric Color:")
        self.t3c2ar11.grid(row=11,column=2,columnspan=1,sticky=tk.N+tk.S+tk.E+tk.W,padx=5,pady=5,ipadx=5,ipady=5)
        self.t3c2br11 = ttk.Label(self.tab3, anchor=tk.S, text="N/A")
        self.t3c2br11.grid(row=11,column=3,columnspan=1,sticky=tk.N+tk.S+tk.E+tk.W,padx=5,pady=5,ipadx=5,ipady=5)
        self.t3c2r12 = tk.Radiobutton(self.tab3,value=2,variable=self.submissionAlphanumericColor,command=self.selectAlphanumericColor)
        self.t3c2r12.configure(foreground="#5c616c",background="#f5f6f7",highlightthickness=0,anchor=tk.N)
        self.t3c2r12.grid(row=12,column=2,columnspan=2,sticky=tk.N+tk.E+tk.W+tk.S,padx=5,pady=5,ipadx=5,ipady=5)
        self.t3c2ar13 = ttk.Label(self.tab3, anchor=tk.S, text="Orientation:")
        self.t3c2ar13.grid(row=13,column=2,columnspan=1,sticky=tk.N+tk.S+tk.E+tk.W,padx=5,pady=5,ipadx=5,ipady=5)
        self.t3c2br13 = ttk.Label(self.tab3, anchor=tk.S, text="N/A")
        self.t3c2br13.grid(row=13,column=3,columnspan=1,sticky=tk.N+tk.S+tk.E+tk.W,padx=5,pady=5,ipadx=5,ipady=5)
        self.t3c2r14 = tk.Radiobutton(self.tab3,value=2,variable=self.submissionOrientation,command=self.selectOrientation)
        self.t3c2r14.configure(foreground="#5c616c",background="#f5f6f7",highlightthickness=0,anchor=tk.N)
        self.t3c2r14.grid(row=14,column=2,columnspan=2,sticky=tk.N+tk.E+tk.W+tk.S,padx=5,pady=5,ipadx=5,ipady=5)
        self.t3c2ar15 = ttk.Label(self.tab3, anchor=tk.CENTER, text="Target Type:")
        self.t3c2ar15.grid(row=15,column=2,columnspan=1,sticky=tk.N+tk.S+tk.E+tk.W,padx=5,pady=5,ipadx=5,ipady=5)
        self.t3c2br15 = ttk.Label(self.tab3, anchor=tk.CENTER, text="N/A")
        self.t3c2br15.grid(row=15,column=3,columnspan=1,sticky=tk.N+tk.S+tk.E+tk.W,padx=5,pady=5,ipadx=5,ipady=5)
        self.t3c2r17 = ttk.Label(self.tab3, anchor=tk.CENTER, text="Description:")
        self.t3c2r17.grid(row=17,column=2,columnspan=2,sticky=tk.N+tk.S+tk.E+tk.W,padx=5,pady=5,ipadx=5,ipady=5)
        self.t3c2r18 = ttk.Label(self.tab3, anchor=tk.CENTER, text="N/A")
        self.t3c2r18.grid(row=18,column=2,columnspan=2,sticky=tk.N+tk.S+tk.E+tk.W,padx=5,pady=5,ipadx=5,ipady=5)
        self.t3c2r19 = tk.Radiobutton(self.tab3,value=2,variable=self.submissionDescription,command=self.selectDescription)
        self.t3c2r19.configure(foreground="#5c616c",background="#f5f6f7",highlightthickness=0,anchor=tk.N)
        self.t3c2r19.grid(row=19,column=2,columnspan=2,sticky=tk.N+tk.E+tk.W+tk.S,padx=5,pady=5,ipadx=5,ipady=5)
        self.t3c2b1 = ttk.Button(self.tab3, text="Delete Classification",command=self.deleteClassification2)
        self.t3c2b1.grid(row=20,column=2,columnspan=2,sticky=tk.N+tk.S+tk.E+tk.W,padx=5,pady=5,ipadx=5,ipady=5)


        # Column Three
        self.t3c3title = ttk.Label(self.tab3, anchor=tk.CENTER, text='Pic 3')
        self.t3c3title.grid(row=1,column=4,columnspan=2,sticky=tk.N+tk.S+tk.E+tk.W,padx=5,pady=5,ipadx=5,ipady=5)
        self.t3c3i1_im = self.t3_default_im.copy()
        self.t3c3i1_tk = tab_tools.im2tk(self.t3c3i1_im)
        self.t3c3i1_org_width,self.t3c3i1_org_height = self.t3c3i1_im.size
        self.t3c3i1 = ttk.Label(self.tab3, anchor=tk.CENTER,image=self.t3c3i1_tk)
        self.t3c3i1.image = self.t3c3i1_tk
        self.t3c3i1.grid(row=3,column=4,rowspan=1,columnspan=2,sticky=tk.N+tk.S+tk.E+tk.W,padx=5,pady=5,ipadx=5,ipady=5)
        # Characteristics
        self.t3c3r4 = tk.Radiobutton(self.tab3,value=3,variable=self.submissionImage,command=self.selectImage)
        self.t3c3r4.configure(foreground="#5c616c",background="#f5f6f7",highlightthickness=0,anchor=tk.N)
        self.t3c3r4.grid(row=4,column=4,columnspan=2,sticky=tk.N+tk.E+tk.W+tk.S,padx=5,pady=5,ipadx=5,ipady=5)
        self.t3c3ar5 = ttk.Label(self.tab3, anchor=tk.CENTER, text="Shape:")
        self.t3c3ar5.grid(row=5,column=4,columnspan=1,sticky=tk.N+tk.S+tk.E+tk.W,padx=5,pady=5,ipadx=5,ipady=5)
        self.t3c3br5 = ttk.Label(self.tab3, anchor=tk.CENTER, text="N/A")
        self.t3c3br5.grid(row=5,column=5,columnspan=1,sticky=tk.N+tk.S+tk.E+tk.W,padx=5,pady=5,ipadx=5,ipady=5)
        self.t3c3ar7 = ttk.Label(self.tab3, anchor=tk.S, text="Background Color:")
        self.t3c3ar7.grid(row=7,column=4,columnspan=1,sticky=tk.N+tk.S+tk.E+tk.W,padx=5,pady=5,ipadx=5,ipady=5)
        self.t3c3br7 = ttk.Label(self.tab3, anchor=tk.S, text="N/A")
        self.t3c3br7.grid(row=7,column=5,columnspan=1,sticky=tk.N+tk.S+tk.E+tk.W,padx=5,pady=5,ipadx=5,ipady=5)
        self.t3c3r8 = tk.Radiobutton(self.tab3,value=3,variable=self.submissionBackgroundColor,command=self.selectBackgroundColor)
        self.t3c3r8.configure(foreground="#5c616c",background="#f5f6f7",highlightthickness=0,anchor=tk.N)
        self.t3c3r8.grid(row=8,column=4,columnspan=2,sticky=tk.N+tk.E+tk.W+tk.S,padx=5,pady=5,ipadx=5,ipady=5)
        self.t3c3ar9 = ttk.Label(self.tab3, anchor=tk.CENTER, text="Alphanumeric:")
        self.t3c3ar9.grid(row=9,column=4,columnspan=1,sticky=tk.N+tk.S+tk.E+tk.W,padx=5,pady=5,ipadx=5,ipady=5)
        self.t3c3br9 = ttk.Label(self.tab3, anchor=tk.CENTER, text="N/A")
        self.t3c3br9.grid(row=9,column=5,columnspan=1,sticky=tk.N+tk.S+tk.E+tk.W,padx=5,pady=5,ipadx=5,ipady=5)
        self.t3c3ar11 = ttk.Label(self.tab3, anchor=tk.S, text="Alphanumeric Color:")
        self.t3c3ar11.grid(row=11,column=4,columnspan=1,sticky=tk.N+tk.S+tk.E+tk.W,padx=5,pady=5,ipadx=5,ipady=5)
        self.t3c3br11 = ttk.Label(self.tab3, anchor=tk.S, text="N/A")
        self.t3c3br11.grid(row=11,column=5,columnspan=1,sticky=tk.N+tk.S+tk.E+tk.W,padx=5,pady=5,ipadx=5,ipady=5)
        self.t3c3r12 = tk.Radiobutton(self.tab3,value=3,variable=self.submissionAlphanumericColor,command=self.selectAlphanumericColor)
        self.t3c3r12.configure(foreground="#5c616c",background="#f5f6f7",highlightthickness=0,anchor=tk.N)
        self.t3c3r12.grid(row=12,column=4,columnspan=2,sticky=tk.N+tk.E+tk.W+tk.S,padx=5,pady=5,ipadx=5,ipady=5)
        self.t3c3ar13 = ttk.Label(self.tab3, anchor=tk.S, text="Orientation:")
        self.t3c3ar13.grid(row=13,column=4,columnspan=1,sticky=tk.N+tk.S+tk.E+tk.W,padx=5,pady=5,ipadx=5,ipady=5)
        self.t3c3br13 = ttk.Label(self.tab3, anchor=tk.S, text="N/A")
        self.t3c3br13.grid(row=13,column=5,columnspan=1,sticky=tk.N+tk.S+tk.E+tk.W,padx=5,pady=5,ipadx=5,ipady=5)
        self.t3c3r14 = tk.Radiobutton(self.tab3,value=3,variable=self.submissionOrientation,command=self.selectOrientation)
        self.t3c3r14.configure(foreground="#5c616c",background="#f5f6f7",highlightthickness=0,anchor=tk.N)
        self.t3c3r14.grid(row=14,column=4,columnspan=2,sticky=tk.N+tk.E+tk.W+tk.S,padx=5,pady=5,ipadx=5,ipady=5)
        self.t3c3ar15 = ttk.Label(self.tab3, anchor=tk.CENTER, text="Target Type:")
        self.t3c3ar15.grid(row=15,column=4,columnspan=1,sticky=tk.N+tk.S+tk.E+tk.W,padx=5,pady=5,ipadx=5,ipady=5)
        self.t3c3br15 = ttk.Label(self.tab3, anchor=tk.CENTER, text="N/A")
        self.t3c3br15.grid(row=15,column=5,columnspan=1,sticky=tk.N+tk.S+tk.E+tk.W,padx=5,pady=5,ipadx=5,ipady=5)
        self.t3c3r17 = ttk.Label(self.tab3, anchor=tk.CENTER, text="Description:")
        self.t3c3r17.grid(row=17,column=4,columnspan=2,sticky=tk.N+tk.S+tk.E+tk.W,padx=5,pady=5,ipadx=5,ipady=5)
        self.t3c3r18 = ttk.Label(self.tab3, anchor=tk.CENTER, text="N/A")
        self.t3c3r18.grid(row=18,column=4,columnspan=2,sticky=tk.N+tk.S+tk.E+tk.W,padx=5,pady=5,ipadx=5,ipady=5)
        self.t3c3r19 = tk.Radiobutton(self.tab3,value=3,variable=self.submissionDescription,command=self.selectDescription)
        self.t3c3r19.configure(foreground="#5c616c",background="#f5f6f7",highlightthickness=0,anchor=tk.N)
        self.t3c3r19.grid(row=19,column=4,columnspan=2,sticky=tk.N+tk.E+tk.W+tk.S,padx=5,pady=5,ipadx=5,ipady=5)
        self.t3c3b1 = ttk.Button(self.tab3, text="Delete Classification",command=self.deleteClassification3)
        self.t3c3b1.grid(row=20,column=4,columnspan=2,sticky=tk.N+tk.S+tk.E+tk.W,padx=5,pady=5,ipadx=5,ipady=5)


        # Column Four
        self.t3c4title = ttk.Label(self.tab3, anchor=tk.CENTER, text='Pic 4')
        self.t3c4title.grid(row=1,column=6,columnspan=2,sticky=tk.N+tk.S+tk.E+tk.W,padx=5,pady=5,ipadx=5,ipady=5)
        self.t3c4i1_im = self.t3_default_im.copy()
        self.t3c4i1_tk = tab_tools.im2tk(self.t3c4i1_im)
        self.t3c4i1_org_width,self.t3c4i1_org_height = self.t3c4i1_im.size
        self.t3c4i1 = ttk.Label(self.tab3, anchor=tk.CENTER,image=self.t3c4i1_tk)
        self.t3c4i1.image = self.t3c4i1_tk
        self.t3c4i1.grid(row=3,column=6,rowspan=1,columnspan=2,sticky=tk.N+tk.S+tk.E+tk.W,padx=5,pady=5,ipadx=5,ipady=5)
        # Characteristics
        self.t3c4r4 = tk.Radiobutton(self.tab3,value=4,variable=self.submissionImage,command=self.selectImage)
        self.t3c4r4.configure(foreground="#5c616c",background="#f5f6f7",highlightthickness=0,anchor=tk.N)
        self.t3c4r4.grid(row=4,column=6,columnspan=2,sticky=tk.N+tk.E+tk.W+tk.S,padx=5,pady=5,ipadx=5,ipady=5)
        self.t3c4ar5 = ttk.Label(self.tab3, anchor=tk.CENTER, text="Shape:")
        self.t3c4ar5.grid(row=5,column=6,columnspan=1,sticky=tk.N+tk.S+tk.E+tk.W,padx=5,pady=5,ipadx=5,ipady=5)
        self.t3c4br5 = ttk.Label(self.tab3, anchor=tk.CENTER, text="N/A")
        self.t3c4br5.grid(row=5,column=7,columnspan=1,sticky=tk.N+tk.S+tk.E+tk.W,padx=5,pady=5,ipadx=5,ipady=5)
        self.t3c4ar7 = ttk.Label(self.tab3, anchor=tk.S, text="Background Color:")
        self.t3c4ar7.grid(row=7,column=6,columnspan=1,sticky=tk.N+tk.S+tk.E+tk.W,padx=5,pady=5,ipadx=5,ipady=5)
        self.t3c4br7 = ttk.Label(self.tab3, anchor=tk.S, text="N/A")
        self.t3c4br7.grid(row=7,column=7,columnspan=1,sticky=tk.N+tk.S+tk.E+tk.W,padx=5,pady=5,ipadx=5,ipady=5)
        self.t3c4r8 = tk.Radiobutton(self.tab3,value=4,variable=self.submissionBackgroundColor,command=self.selectBackgroundColor)
        self.t3c4r8.configure(foreground="#5c616c",background="#f5f6f7",highlightthickness=0,anchor=tk.N)
        self.t3c4r8.grid(row=8,column=6,columnspan=2,sticky=tk.N+tk.E+tk.W+tk.S,padx=5,pady=5,ipadx=5,ipady=5)
        self.t3c4ar9 = ttk.Label(self.tab3, anchor=tk.CENTER, text="Alphanumeric:")
        self.t3c4ar9.grid(row=9,column=6,columnspan=1,sticky=tk.N+tk.S+tk.E+tk.W,padx=5,pady=5,ipadx=5,ipady=5)
        self.t3c4br9 = ttk.Label(self.tab3, anchor=tk.CENTER, text="N/A")
        self.t3c4br9.grid(row=9,column=7,columnspan=1,sticky=tk.N+tk.S+tk.E+tk.W,padx=5,pady=5,ipadx=5,ipady=5)
        self.t3c4ar11 = ttk.Label(self.tab3, anchor=tk.S, text="Alphanumeric Color:")
        self.t3c4ar11.grid(row=11,column=6,columnspan=1,sticky=tk.N+tk.S+tk.E+tk.W,padx=5,pady=5,ipadx=5,ipady=5)
        self.t3c4br11 = ttk.Label(self.tab3, anchor=tk.S, text="N/A")
        self.t3c4br11.grid(row=11,column=7,columnspan=1,sticky=tk.N+tk.S+tk.E+tk.W,padx=5,pady=5,ipadx=5,ipady=5)
        self.t3c4r12 = tk.Radiobutton(self.tab3,value=4,variable=self.submissionAlphanumericColor,command=self.selectAlphanumericColor)
        self.t3c4r12.configure(foreground="#5c616c",background="#f5f6f7",highlightthickness=0,anchor=tk.N)
        self.t3c4r12.grid(row=12,column=6,columnspan=2,sticky=tk.N+tk.E+tk.W+tk.S,padx=5,pady=5,ipadx=5,ipady=5)
        self.t3c4ar13 = ttk.Label(self.tab3, anchor=tk.S, text="Orientation:")
        self.t3c4ar13.grid(row=13,column=6,columnspan=1,sticky=tk.N+tk.S+tk.E+tk.W,padx=5,pady=5,ipadx=5,ipady=5)
        self.t3c4br13 = ttk.Label(self.tab3, anchor=tk.S, text="N/A")
        self.t3c4br13.grid(row=13,column=7,columnspan=1,sticky=tk.N+tk.S+tk.E+tk.W,padx=5,pady=5,ipadx=5,ipady=5)
        self.t3c4r14 = tk.Radiobutton(self.tab3,value=4,variable=self.submissionOrientation,command=self.selectOrientation)
        self.t3c4r14.configure(foreground="#5c616c",background="#f5f6f7",highlightthickness=0,anchor=tk.N)
        self.t3c4r14.grid(row=14,column=6,columnspan=2,sticky=tk.N+tk.E+tk.W+tk.S,padx=5,pady=5,ipadx=5,ipady=5)
        self.t3c4ar15 = ttk.Label(self.tab3, anchor=tk.CENTER, text="Target Type:")
        self.t3c4ar15.grid(row=15,column=6,columnspan=1,sticky=tk.N+tk.S+tk.E+tk.W,padx=5,pady=5,ipadx=5,ipady=5)
        self.t3c4br15 = ttk.Label(self.tab3, anchor=tk.CENTER, text="N/A")
        self.t3c4br15.grid(row=15,column=7,columnspan=1,sticky=tk.N+tk.S+tk.E+tk.W,padx=5,pady=5,ipadx=5,ipady=5)
        self.t3c4r17 = ttk.Label(self.tab3, anchor=tk.CENTER, text="Description:")
        self.t3c4r17.grid(row=17,column=6,columnspan=2,sticky=tk.N+tk.S+tk.E+tk.W,padx=5,pady=5,ipadx=5,ipady=5)
        self.t3c4r18 = ttk.Label(self.tab3, anchor=tk.CENTER, text="N/A")
        self.t3c4r18.grid(row=18,column=6,columnspan=2,sticky=tk.N+tk.S+tk.E+tk.W,padx=5,pady=5,ipadx=5,ipady=5)
        self.t3c4r19 = tk.Radiobutton(self.tab3,value=4,variable=self.submissionDescription,command=self.selectDescription)
        self.t3c4r19.configure(foreground="#5c616c",background="#f5f6f7",highlightthickness=0,anchor=tk.N)
        self.t3c4r19.grid(row=19,column=6,columnspan=2,sticky=tk.N+tk.E+tk.W+tk.S,padx=5,pady=5,ipadx=5,ipady=5)
        self.t3c4b1 = ttk.Button(self.tab3, text="Delete Classification",command=self.deleteClassification4)
        self.t3c4b1.grid(row=20,column=6,columnspan=2,sticky=tk.N+tk.S+tk.E+tk.W,padx=5,pady=5,ipadx=5,ipady=5)



        # Column Five
        self.t3c5title = ttk.Label(self.tab3, anchor=tk.CENTER, text='Pic 5')
        self.t3c5title.grid(row=1,column=8,columnspan=2,sticky=tk.N+tk.S+tk.E+tk.W,padx=5,pady=5,ipadx=5,ipady=5)
        self.t3c5i1_im = self.t3_default_im.copy()
        self.t3c5i1_tk = tab_tools.im2tk(self.t3c5i1_im)
        self.t3c5i1_org_width,self.t3c5i1_org_height = self.t3c5i1_im.size
        self.t3c5i1 = ttk.Label(self.tab3, anchor=tk.CENTER,image=self.t3c5i1_tk)
        self.t3c5i1.image = self.t3c5i1_tk
        self.t3c5i1.grid(row=3,column=8,rowspan=1,columnspan=2,sticky=tk.N+tk.S+tk.E+tk.W,padx=5,pady=5,ipadx=5,ipady=5)
        # Characteristics
        self.t3c5r4 = tk.Radiobutton(self.tab3,value=5,variable=self.submissionImage,command=self.selectImage)
        self.t3c5r4.configure(foreground="#5c616c",background="#f5f6f7",highlightthickness=0,anchor=tk.N)
        self.t3c5r4.grid(row=4,column=8,columnspan=2,sticky=tk.N+tk.E+tk.W+tk.S,padx=5,pady=5,ipadx=5,ipady=5)
        self.t3c5ar5 = ttk.Label(self.tab3, anchor=tk.CENTER, text="Shape:")
        self.t3c5ar5.grid(row=5,column=8,columnspan=1,sticky=tk.N+tk.S+tk.E+tk.W,padx=5,pady=5,ipadx=5,ipady=5)
        self.t3c5br5 = ttk.Label(self.tab3, anchor=tk.CENTER, text="N/A")
        self.t3c5br5.grid(row=5,column=9,columnspan=1,sticky=tk.N+tk.S+tk.E+tk.W,padx=5,pady=5,ipadx=5,ipady=5)
        self.t3c5ar7 = ttk.Label(self.tab3, anchor=tk.S, text="Background Color:")
        self.t3c5ar7.grid(row=7,column=8,columnspan=1,sticky=tk.N+tk.S+tk.E+tk.W,padx=5,pady=5,ipadx=5,ipady=5)
        self.t3c5br7 = ttk.Label(self.tab3, anchor=tk.S, text="N/A")
        self.t3c5br7.grid(row=7,column=9,columnspan=1,sticky=tk.N+tk.S+tk.E+tk.W,padx=5,pady=5,ipadx=5,ipady=5)
        self.t3c5r8 = tk.Radiobutton(self.tab3,value=5,variable=self.submissionBackgroundColor,command=self.selectBackgroundColor)
        self.t3c5r8.configure(foreground="#5c616c",background="#f5f6f7",highlightthickness=0,anchor=tk.N)
        self.t3c5r8.grid(row=8,column=8,columnspan=2,sticky=tk.N+tk.E+tk.W+tk.S,padx=5,pady=5,ipadx=5,ipady=5)
        self.t3c5ar9 = ttk.Label(self.tab3, anchor=tk.CENTER, text="Alphanumeric:")
        self.t3c5ar9.grid(row=9,column=8,columnspan=1,sticky=tk.N+tk.S+tk.E+tk.W,padx=5,pady=5,ipadx=5,ipady=5)
        self.t3c5br9 = ttk.Label(self.tab3, anchor=tk.CENTER, text="N/A")
        self.t3c5br9.grid(row=9,column=9,columnspan=1,sticky=tk.N+tk.S+tk.E+tk.W,padx=5,pady=5,ipadx=5,ipady=5)
        self.t3c5ar11 = ttk.Label(self.tab3, anchor=tk.S, text="Alphanumeric Color:")
        self.t3c5ar11.grid(row=11,column=8,columnspan=1,sticky=tk.N+tk.S+tk.E+tk.W,padx=5,pady=5,ipadx=5,ipady=5)
        self.t3c5br11 = ttk.Label(self.tab3, anchor=tk.S, text="N/A")
        self.t3c5br11.grid(row=11,column=9,columnspan=1,sticky=tk.N+tk.S+tk.E+tk.W,padx=5,pady=5,ipadx=5,ipady=5)
        self.t3c5r12 = tk.Radiobutton(self.tab3,value=5,variable=self.submissionAlphanumericColor,command=self.selectAlphanumericColor)
        self.t3c5r12.configure(foreground="#5c616c",background="#f5f6f7",highlightthickness=0,anchor=tk.N)
        self.t3c5r12.grid(row=12,column=8,columnspan=2,sticky=tk.N+tk.E+tk.W+tk.S,padx=5,pady=5,ipadx=5,ipady=5)
        self.t3c5ar13 = ttk.Label(self.tab3, anchor=tk.S, text="Orientation:")
        self.t3c5ar13.grid(row=13,column=8,columnspan=1,sticky=tk.N+tk.S+tk.E+tk.W,padx=5,pady=5,ipadx=5,ipady=5)
        self.t3c5br13 = ttk.Label(self.tab3, anchor=tk.S, text="N/A")
        self.t3c5br13.grid(row=13,column=9,columnspan=1,sticky=tk.N+tk.S+tk.E+tk.W,padx=5,pady=5,ipadx=5,ipady=5)
        self.t3c5r14 = tk.Radiobutton(self.tab3,value=5,variable=self.submissionOrientation,command=self.selectOrientation)
        self.t3c5r14.configure(foreground="#5c616c",background="#f5f6f7",highlightthickness=0,anchor=tk.N)
        self.t3c5r14.grid(row=14,column=8,columnspan=2,sticky=tk.N+tk.E+tk.W+tk.S,padx=5,pady=5,ipadx=5,ipady=5)
        self.t3c5ar15 = ttk.Label(self.tab3, anchor=tk.CENTER, text="Target Type:")
        self.t3c5ar15.grid(row=15,column=8,columnspan=1,sticky=tk.N+tk.S+tk.E+tk.W,padx=5,pady=5,ipadx=5,ipady=5)
        self.t3c5br15 = ttk.Label(self.tab3, anchor=tk.CENTER, text="N/A")
        self.t3c5br15.grid(row=15,column=9,columnspan=1,sticky=tk.N+tk.S+tk.E+tk.W,padx=5,pady=5,ipadx=5,ipady=5)
        self.t3c5r17 = ttk.Label(self.tab3, anchor=tk.CENTER, text="Description:")
        self.t3c5r17.grid(row=17,column=8,columnspan=2,sticky=tk.N+tk.S+tk.E+tk.W,padx=5,pady=5,ipadx=5,ipady=5)
        self.t3c5r18 = ttk.Label(self.tab3, anchor=tk.CENTER, text="N/A")
        self.t3c5r18.grid(row=18,column=8,columnspan=2,sticky=tk.N+tk.S+tk.E+tk.W,padx=5,pady=5,ipadx=5,ipady=5)
        self.t3c5r19 = tk.Radiobutton(self.tab3,value=5,variable=self.submissionDescription,command=self.selectDescription)
        self.t3c5r19.configure(foreground="#5c616c",background="#f5f6f7",highlightthickness=0,anchor=tk.N)
        self.t3c5r19.grid(row=19,column=8,columnspan=2,sticky=tk.N+tk.E+tk.W+tk.S,padx=5,pady=5,ipadx=5,ipady=5)
        self.t3c5b1 = ttk.Button(self.tab3, text="Delete Classification",command=self.deleteClassification5)
        self.t3c5b1.grid(row=20,column=8,columnspan=2,sticky=tk.N+tk.S+tk.E+tk.W,padx=5,pady=5,ipadx=5,ipady=5)


        # Column Six
        self.t3sep56 = ttk.Separator(self.tab3, orient=tk.VERTICAL)
        self.t3sep56.grid(row=1, column=10, rowspan=20,sticky=tk.N+tk.S+tk.E+tk.W, pady=5)
        self.t3c6title = ttk.Label(self.tab3, anchor=tk.CENTER, text='To Submit')
        self.t3c6title.grid(row=1,column=10,columnspan=2,sticky=tk.N+tk.S+tk.E+tk.W,padx=5,pady=5,ipadx=5,ipady=5)
        self.t3c6i1_im = self.t3_default_im.copy()
        self.t3c6i1_tk = tab_tools.im2tk(self.t3c6i1_im)
        self.t3c6i1_org_width,self.t3c6i1_org_height = self.t3c6i1_im.size
        self.t3c6i1 = ttk.Label(self.tab3, anchor=tk.CENTER,image=self.t3c6i1_tk)
        self.t3c6i1.image = self.t3c6i1_tk
        self.t3c6i1.grid(row=3,column=10,rowspan=1,columnspan=2,sticky=tk.N+tk.S+tk.E+tk.W,padx=5,pady=5,ipadx=5,ipady=5)
        # Characteristics
        self.t3c6ar5 = ttk.Label(self.tab3, anchor=tk.CENTER, text="Shape:")
        self.t3c6ar5.grid(row=5,column=10,columnspan=1,sticky=tk.N+tk.S+tk.E+tk.W,padx=5,pady=5,ipadx=5,ipady=5)
        self.t3c6br5 = ttk.Label(self.tab3, anchor=tk.CENTER, text="N/A")
        self.t3c6br5.grid(row=5,column=11,columnspan=1,sticky=tk.N+tk.S+tk.E+tk.W,padx=5,pady=5,ipadx=5,ipady=5)
        self.t3c6ar7 = ttk.Label(self.tab3, anchor=tk.CENTER, text="Background Color:")
        self.t3c6ar7.grid(row=7,column=10,columnspan=1,sticky=tk.N+tk.S+tk.E+tk.W,padx=5,pady=5,ipadx=5,ipady=5)
        self.t3c6br7 = ttk.Label(self.tab3, anchor=tk.CENTER, text="N/A")
        self.t3c6br7.grid(row=7,column=11,columnspan=1,sticky=tk.N+tk.S+tk.E+tk.W,padx=5,pady=5,ipadx=5,ipady=5)
        self.t3c6ar9 = ttk.Label(self.tab3, anchor=tk.CENTER, text="Alphanumeric:")
        self.t3c6ar9.grid(row=9,column=10,columnspan=1,sticky=tk.N+tk.S+tk.E+tk.W,padx=5,pady=5,ipadx=5,ipady=5)
        self.t3c6br9 = ttk.Label(self.tab3, anchor=tk.CENTER, text="N/A")
        self.t3c6br9.grid(row=9,column=11,columnspan=1,sticky=tk.N+tk.S+tk.E+tk.W,padx=5,pady=5,ipadx=5,ipady=5)
        self.t3c6ar11 = ttk.Label(self.tab3, anchor=tk.CENTER, text="Alphanumeric Color:")
        self.t3c6ar11.grid(row=11,column=10,columnspan=1,sticky=tk.N+tk.S+tk.E+tk.W,padx=5,pady=5,ipadx=5,ipady=5)
        self.t3c6br11 = ttk.Label(self.tab3, anchor=tk.CENTER, text="N/A")
        self.t3c6br11.grid(row=11,column=11,columnspan=1,sticky=tk.N+tk.S+tk.E+tk.W,padx=5,pady=5,ipadx=5,ipady=5)
        self.t3c6ar13 = ttk.Label(self.tab3, anchor=tk.CENTER, text="Orientation:")
        self.t3c6ar13.grid(row=13,column=10,columnspan=1,sticky=tk.N+tk.S+tk.E+tk.W,padx=5,pady=5,ipadx=5,ipady=5)
        self.t3c6br13 = ttk.Label(self.tab3, anchor=tk.CENTER, text="N/A")
        self.t3c6br13.grid(row=13,column=11,columnspan=1,sticky=tk.N+tk.S+tk.E+tk.W,padx=5,pady=5,ipadx=5,ipady=5)
        self.t3c6ar15 = ttk.Label(self.tab3, anchor=tk.CENTER, text="Target Type:")
        self.t3c6ar15.grid(row=15,column=10,columnspan=1,sticky=tk.N+tk.S+tk.E+tk.W,padx=5,pady=5,ipadx=5,ipady=5)
        self.t3c6br15 = ttk.Label(self.tab3, anchor=tk.CENTER, text="N/A")
        self.t3c6br15.grid(row=15,column=11,columnspan=1,sticky=tk.N+tk.S+tk.E+tk.W,padx=5,pady=5,ipadx=5,ipady=5)
        self.t3c6r17 = ttk.Label(self.tab3, anchor=tk.CENTER, text="Description:")
        self.t3c6r17.grid(row=17,column=10,columnspan=2,sticky=tk.N+tk.S+tk.E+tk.W,padx=5,pady=5,ipadx=5,ipady=5)
        self.t3c6r18 = ttk.Label(self.tab3, anchor=tk.CENTER, text="N/A")
        self.t3c6r18.grid(row=18,column=10,columnspan=2,sticky=tk.N+tk.S+tk.E+tk.W,padx=5,pady=5,ipadx=5,ipady=5)
        # Submit button
        self.t3c6b1 = ttk.Button(self.tab3, text="Submit Target",command=self.submitTarget)
        self.t3c6b1.grid(row=20,column=10,columnspan=2,sticky=tk.N+tk.S+tk.E+tk.W,padx=5,pady=5,ipadx=5,ipady=5)


        # TAB 4: AUTONOMOUS TARGET SUBMISSION ------------------------------------------------
        self.tab4 = ttk.Frame(self.n)
        self.n.add(self.tab4, text='Autonomous Target Submission')

        # Done with initialization
        self.initialized = True

        # KEY BINDINGS ------------------------------------------------------------
        self.master.bind("<<NotebookTabChanged>>",self.tabChanged)


    def nextCropped(self,event=None):
        """
        Requests and displays next cropped image

        @type  event: event
        @param event: Right arrow event

        @rtype:  None
        @return: None
        """
        if not(self.t2_entry_focus):
            self.serverConnected = self.interface.ping()
            if self.serverConnected:
                time0 = time.time()
                query = self.interface.getNextCroppedImage()
                if query == None:
                    self.t2_functional = False
                    self.noNextCropped()
                else:
                    self.t2_functional = True
                    self.imageID = query[1]
                    self.cropped_np = np.array(query[0])
                    yaw_angle = self.getYawAngle(self.imageID)
                    self.cropped_np = imutils.rotate_bound(self.cropped_np,yaw_angle)
                    status = query[2]
                    if status:
                        self.t2c2lr48b.configure(text='submitted',foreground='green')
                    else:
                        self.t2c2lr48b.configure(text='unsubmitted',foreground='red')
                time1 = time.time()
                self.cropped_im = tab_tools.np2im(self.cropped_np)
                self.cropped_width,self.cropped_height = self.cropped_im.size
                self.cropped_resized_im = tab_tools.resizeIm(self.cropped_im,self.cropped_width,self.cropped_height,self.t2c2i1_width,self.t2c2i1_height)
                self.cropped_tk = tab_tools.im2tk(self.cropped_resized_im)
                self.t2c2i1.configure(image=self.cropped_tk)

                time2 = time.time()
                #print("server request = ",time1-time0,"gui = ",time2-time1)

    def previousCropped(self,event):
        """
        Requests and displays previous cropped image

        @type  event: event
        @param event: Left arrow event

        @rtype:  None
        @return: None
        """
        if not(self.t2_entry_focus):
            focus = self.tab2.focus_get()
            self.serverConnected = self.interface.ping()
            if self.serverConnected:
                time0 = time.time()
                query = self.interface.getPrevCroppedImage()
                if query == None:
                    self.t2_functional = False
                    self.noPreviousCropped()
                else:
                    self.t2_functional = True
                    self.imageID = query[1]
                    self.cropped_np = np.array(query[0])
                    yaw_angle = self.getYawAngle(self.imageID)
                    self.cropped_np = imutils.rotate_bound(self.cropped_np,yaw_angle)
                    status = query[2]
                    if status:
                        self.t2c2lr48b.configure(text='submitted',foreground='green')
                    else:
                        self.t2c2lr48b.configure(text='unsubmitted',foreground='red')
                time1 = time.time()
                self.cropped_im = tab_tools.np2im(self.cropped_np)
                self.cropped_width,self.cropped_height = self.cropped_im.size
                self.cropped_resized_im = tab_tools.resizeIm(self.cropped_im,self.cropped_width,self.cropped_height,self.t2c2i1_width,self.t2c2i1_height)
                self.cropped_tk = tab_tools.im2tk(self.cropped_resized_im)
                self.t2c2i1.configure(image=self.cropped_tk)
                time2 = time.time()
                #print("server request = ",time1-time0,"gui = ",time2-time1)


    def submitClassification(self,event=None):
        """
        Submits classification of image to server

        @type  event: event
        @param event: Enter press event

        @rtype:  None
        @return: None
        """
        if self.t2_functional:
            type = self.t2c2l14_var.get()
            if type == 'emergent':
                description = self.t2c2l16_var.get()
                classification = client_rest.Classification(self.imageID,type,desc = description)
                self.interface.postClass(classification)
                self.entry_focus_out()
                self.nextCropped()
            else:
                alphanumeric = self.t2c2l4.get()
                if alphanumeric != "":
                    shape = self.t2c2l2_var.get()
                    orientation = self.t2c2l6_var.get()
                    background_color = self.t2c2l10_var.get()
                    alpha_color = self.t2c2l12_var.get()
                    classification = client_rest.Classification(self.imageID,type,orientation=orientation,shape=shape,bgColor=background_color,alpha=alphanumeric,alphaColor=alpha_color)
                    self.interface.postClass(classification)
                    self.entry_focus_out()
                    self.nextCropped()

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
            self.tab0.run()
        else:
            self.interface = self.tab0.interfaceCall()

        if active_tab == 1:
            self.tab1.run(self.interface)

        if active_tab == 2:
            self.resizeEventTab2()
            self.master.bind("<Right>",self.nextCropped)
            self.master.bind("<Left>",self.previousCropped)
            self.master.unbind("<d>")
            self.master.unbind("<a>")
            self.master.bind("<Configure>",self.resizeEventTab2)
            self.master.unbind("<Control-z>")
            self.master.bind("<Return>",self.submitClassification)
            self.t2c2l16.bind("<FocusIn>",self.entry_focus_in)
            self.t2c2l16.bind("<FocusOut>",self.entry_focus_out)
            self.t2c2l16.bind("<Leave>",self.entry_focus_out)
        else:
            self.t2c2l16.unbind("<FocusIn>")
            self.t2c2l16.unbind("<FocusOut>")
            self.t2c2l16.unbind("<Leave>")
        if active_tab == 3:
            self.resizeEventTab3()
            self.master.bind("<Right>",self.nextClassified)
            self.master.bind("<Left>",self.previousClassified)
            self.master.unbind("<d>")
            self.master.unbind("<a>")
            self.master.bind("<Configure>",self.resizeEventTab3)
            self.master.unbind("<Control-z>")
            self.master.bind("<Return>",self.submitTarget)
            self.updateManualSubmissionTab()
        if active_tab == 4:
            self.master.unbind("<Right>")
            self.master.unbind("<Left>")
            self.master.unbind("<d>")
            self.master.unbind("<a>")
            self.master.unbind("<Configure>")
            self.master.unbind("<Control-z>")
            self.master.unbind("<Return>")
        self.master.focus_set()

    def pingServerTab2(self):
        """
        Checks if server is correctly connected

        @rtype:  None
        @return: None
        """
        self.serverConnected = self.interface.ping()
        if self.serverConnected:
            self.cropped_np = tab_tools.get_image('assets/classify_instructions.jpg')
        else:
            self.cropped_np = tab_tools.get_image('assets/server_error.jpg')

    def resizeEventTab2(self,event=None):
        """
        Resizes picture on Tab2
        @type  event: event
        @param event: resize window event

        @rtype:  None
        @return: None
        """
        if self.initialized and (time.time()-self.resize_counter_tab2) > 0.050:
            if self.t2c2i1.winfo_width() > 1:
                self.resize_counter_tab2 = time.time()
                self.master.update()
                self.t2c2i1_width = self.t2c2i1.winfo_width()
                self.t2c2i1_height = self.t2c2i1.winfo_height()
                self.cropped_resized_im = tab_tools.resizeIm(self.cropped_im,self.cropped_width,self.cropped_height,self.t2c2i1_width,self.t2c2i1_height)
                self.cropped_tk = tab_tools.im2tk(self.cropped_resized_im)
                self.t2c2i1.configure(image=self.cropped_tk)
                # resize compass
                '''
                self.t2c3i1_width = self.t2c3i1.winfo_width()
                self.t2c3i1_height = self.t2c3i1.winfo_height()
                resized_im = tab_tools.resizeIm(self.t2c3i1_im,self.t2c3i1_default_width,self.t2c3i1_default_height,self.t2c3i1_width,self.t2c3i1_height)
                self.t2c3i1_tk = tab_tools.im2tk(resized_im)
                self.t2c3i1.configure(image=self.t2c3i1_tk)
                '''

    def resizeEventTab3(self,event=None):
        """
        Resizes picture on Tab3
        @type  event: event
        @param event: resize window event

        @rtype:  None
        @return: None
        """
        if self.initialized and (time.time()-self.resize_counter_tab3) > 0.050:
            if self.t3c1i1.winfo_width() > 1:
                self.resize_counter_tab3 = time.time()
                self.master.update()
                # Col 1 Image
                self.t3c1i1_width = self.t3c1i1.winfo_width()
                self.t3c1i1_height = self.t3c1i1.winfo_height()
                resized_im = tab_tools.resizeIm(self.t3c1i1_im,self.t3c1i1_org_width,self.t3c1i1_org_height,self.t3c1i1_width,self.t3c1i1_height)
                self.t3c1i1_tk = tab_tools.im2tk(resized_im)
                self.t3c1i1.configure(image=self.t3c1i1_tk)
                # Col 2 Image
                self.t3c2i1_width = self.t3c2i1.winfo_width()
                self.t3c2i1_height = self.t3c2i1.winfo_height()
                resized_im = tab_tools.resizeIm(self.t3c2i1_im,self.t3c2i1_org_width,self.t3c2i1_org_height,self.t3c2i1_width,self.t3c2i1_height)
                self.t3c2i1_tk = tab_tools.im2tk(resized_im)
                self.t3c2i1.configure(image=self.t3c2i1_tk)
                # Col 3 Image
                self.t3c3i1_width = self.t3c3i1.winfo_width()
                self.t3c3i1_height = self.t3c3i1.winfo_height()
                resized_im = tab_tools.resizeIm(self.t3c3i1_im,self.t3c3i1_org_width,self.t3c3i1_org_height,self.t3c3i1_width,self.t3c3i1_height)
                self.t3c3i1_tk = tab_tools.im2tk(resized_im)
                self.t3c3i1.configure(image=self.t3c3i1_tk)
                # Col 4 Image
                self.t3c4i1_width = self.t3c4i1.winfo_width()
                self.t3c4i1_height = self.t3c4i1.winfo_height()
                resized_im = tab_tools.resizeIm(self.t3c4i1_im,self.t3c4i1_org_width,self.t3c4i1_org_height,self.t3c4i1_width,self.t3c4i1_height)
                self.t3c4i1_tk = tab_tools.im2tk(resized_im)
                self.t3c4i1.configure(image=self.t3c4i1_tk)
                # Col 5 Image
                self.t3c5i1_width = self.t3c5i1.winfo_width()
                self.t3c5i1_height = self.t3c5i1.winfo_height()
                resized_im = tab_tools.resizeIm(self.t3c5i1_im,self.t3c5i1_org_width,self.t3c5i1_org_height,self.t3c5i1_width,self.t3c5i1_height)
                self.t3c5i1_tk = tab_tools.im2tk(resized_im)
                self.t3c5i1.configure(image=self.t3c5i1_tk)
                # Col 6 Image
                self.t3c6i1_width = self.t3c6i1.winfo_width()
                self.t3c6i1_height = self.t3c6i1.winfo_height()
                resized_im = tab_tools.resizeIm(self.t3c6i1_im,self.t3c6i1_org_width,self.t3c6i1_org_height,self.t3c6i1_width,self.t3c6i1_height)
                self.t3c6i1_tk = tab_tools.im2tk(resized_im)
                self.t3c6i1.configure(image=self.t3c6i1_tk)



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



    def noNextCropped(self):
        """
        Checks if server is correctly connected

        @rtype:  None
        @return: None
        """
        self.serverConnected = self.interface.ping()
        if self.serverConnected:
            self.cropped_np = tab_tools.get_image('assets/noNextCropped.jpg')
            self.t2c2lr48b.configure(text='N/A',foreground='#636363')
        else:
            self.cropped_np = tab_tools.get_image('assets/server_error.jpg')
            self.t2c2lr48b.configure(text='N/A',foreground='#636363')

    def noPreviousCropped(self):
        """
        Checks if server is correctly connected

        @rtype:  None
        @return: None
        """
        self.serverConnected = self.interface.ping()
        if self.serverConnected:
            self.cropped_np = tab_tools.get_image('assets/noPreviousCropped.jpg')
            self.t2c2lr48b.configure(text='N/A',foreground='#636363')
        else:
            self.cropped_np = tab_tools.get_image('assets/server_error.jpg')
            self.t2c2lr48b.configure(text='N/A',foreground='#636363')

    def disableEmergentDescription(self,*args):
        """
        Disables emergent discription unless emergent target selected

        @rtype:  None
        @return: None
        """
        if self.t2c2l14_var.get() == 'emergent':
            self.t2c2l16.configure(state=tk.NORMAL)
            self.t2c2l2.configure(state=tk.DISABLED)
            self.t2c2l4.configure(state=tk.DISABLED)
            self.t2c2l6.configure(state=tk.DISABLED)
            self.t2c2l10.configure(state=tk.DISABLED)
            self.t2c2l12.configure(state=tk.DISABLED)
        else:
            self.t2c2l16_var.set(None)
            self.t2c2l16.configure(state=tk.DISABLED)
            self.t2c2l2.configure(state=tk.NORMAL)
            self.t2c2l4.configure(state=tk.NORMAL)
            self.t2c2l6.configure(state=tk.NORMAL)
            self.t2c2l10.configure(state=tk.NORMAL)
            self.t2c2l12.configure(state=tk.NORMAL)


    def alphanumericChanged(self,*args):
        """
        Fixes if you entered something wrong

        @rtype:  None
        @return: None
        """
        input = self.t2c2l4.get()
        if len(input) != 1:
            print("INPUT ERROR!")

    def alphanumericValidate(self,type, entry):
        """
        Fixes alphanumeric if you entered something wrong

        @rtype:  None
        @return: None
        """
        if type == '1':
        # runs if you try to insert something
            if len(entry) == 1 and (entry.isdigit() or entry.isalpha()):
                if entry.isupper():
                    return True
                else:
                    self.t2c2l4_var.set(entry.upper())
                    return True
            else:
                return False
        else:
            return True

    def updateManualSubmissionTab(self):
        self.serverConnected = self.interface.ping()
        if self.serverConnected:
            self.pendingList = self.interface.getPendingSubmissions()

            if self.pendingList == None:
                pics = 0
                self.t3titleB.configure(text=0)
                self.t3titleD.configure(text=0)
                self.t3c1i1_im = self.t3_default_im.copy()
                self.t3c1i1_tk = tab_tools.im2tk(self.t3c1i1_im)
                self.t3c1i1_org_width,self.t3c1i1_org_height = self.t3c1i1_im.size
                self.t3c1i1.configure(image=self.t3c1i1_tk)
                self.t3c2i1_im = self.t3_default_im.copy()
                self.t3c2i1_tk = tab_tools.im2tk(self.t3c2i1_im)
                self.t3c2i1_org_width,self.t3c2i1_org_height = self.t3c2i1_im.size
                self.t3c2i1.configure(image=self.t3c2i1_tk)
                self.t3c3i1_im = self.t3_default_im.copy()
                self.t3c3i1_tk = tab_tools.im2tk(self.t3c3i1_im)
                self.t3c3i1_org_width,self.t3c3i1_org_height = self.t3c3i1_im.size
                self.t3c3i1.configure(image=self.t3c3i1_tk)
                self.t3c4i1_im = self.t3_default_im.copy()
                self.t3c4i1_tk = tab_tools.im2tk(self.t3c4i1_im)
                self.t3c4i1_org_width,self.t3c4i1_org_height = self.t3c4i1_im.size
                self.t3c4i1.configure(image=self.t3c4i1_tk)
                self.t3c5i1_im = self.t3_default_im.copy()
                self.t3c5i1_tk = tab_tools.im2tk(self.t3c5i1_im)
                self.t3c5i1_org_width,self.t3c5i1_org_height = self.t3c5i1_im.size
                self.t3c5i1.configure(image=self.t3c5i1_tk)
                self.t3c6i1_im = self.t3_default_im.copy()
                self.t3c6i1_tk = tab_tools.im2tk(self.t3c6i1_im)
                self.t3c6i1_org_width,self.t3c6i1_org_height = self.t3c6i1_im.size
                self.t3c6i1.configure(image=self.t3c6i1_tk)
                self.t3c1br5.configure(text="N/A")
                self.t3c1br7.configure(text="N/A")
                self.t3c1br9.configure(text="N/A")
                self.t3c1br11.configure(text="N/A")
                self.t3c1br13.configure(text="N/A")
                self.t3c1br15.configure(text="N/A")
                self.t3c1r18.configure(text="N/A")
                self.t3c1r4.configure(state=tk.DISABLED)
                self.t3c1r8.configure(state=tk.DISABLED)
                self.t3c1r12.configure(state=tk.DISABLED)
                self.t3c1r14.configure(state=tk.DISABLED)
                self.t3c1r19.configure(state=tk.DISABLED)
                self.t3c1b1.configure(state=tk.DISABLED)
                self.t3c6br5.configure(text="N/A")
                self.t3c6br7.configure(text="N/A")
                self.t3c6br9.configure(text="N/A")
                self.t3c6br11.configure(text="N/A")
                self.t3c6br13.configure(text="N/A")
                self.t3c6br15.configure(text="N/A")
                self.t3c6r18.configure(text="N/A")
                self.resizeEventTab3()
            else:
                self.t3_total_targets = len(self.pendingList)
                if self.t3_current_target == 0:
                    self.t3_current_target = 1
                elif self.t3_current_target > self.t3_total_targets:
                    self.t3_current_target = self.t3_total_targets
                self.t3titleD.configure(text=self.t3_total_targets)
                self.t3titleB.configure(text=self.t3_current_target)
                pics = len(self.pendingList[self.t3_current_target-1])
                self.target_id = self.pendingList[self.t3_current_target-1][0].target
                if pics > 5:
                    pics = 5
                # Because of the preceeding if/else statement there will always be at least 1 pic
                query = self.interface.getCroppedImage(self.pendingList[self.t3_current_target-1][0].crop_id)
                self.t3c1i1_im = query[0]
                yaw_angle = self.getYawAngle(query[1])
                self.t3c1i1_im = self.t3c1i1_im.rotate(yaw_angle,expand=1)
                self.t3c1i1_org_width,self.t3c1i1_org_height = self.t3c1i1_im.size
                self.t3c1i1_tk = tab_tools.im2tk(self.t3c1i1_im)
                self.t3c1i1.configure(image=self.t3c1i1_tk)
                display_shape = self.pendingList[self.t3_current_target-1][0].shape
                if display_shape != None:
                    self.t3c1br5.configure(text=display_shape)
                else:
                    self.t3c1br5.configure(text="N/A")
                display_bg_color = self.pendingList[self.t3_current_target-1][0].background_color
                if display_bg_color != None:
                    self.t3c1br7.configure(text=display_bg_color)
                else:
                    self.t3c1br7.configure(text="N/A")
                display_alphanumeric = self.pendingList[self.t3_current_target-1][0].alphanumeric
                if display_alphanumeric != None:
                    self.t3c1br9.configure(text=display_alphanumeric)
                else:
                    self.t3c1br9.configure(text="N/A")
                display_alpha_color = self.pendingList[self.t3_current_target-1][0].alphanumeric_color
                if display_alpha_color != None:
                    self.t3c1br11.configure(text=display_alpha_color)
                else:
                    self.t3c1br11.configure(text="N/A")
                display_orientation = self.pendingList[self.t3_current_target-1][0].orientation
                if display_orientation != None:
                    self.t3c1br13.configure(text=display_orientation)
                else:
                    self.t3c1br13.configure(text="N/A")
                display_decription = self.pendingList[self.t3_current_target-1][0].description
                if display_decription != "":
                    self.t3c1r18.configure(text=display_decription)
                else:
                    self.t3c1r18.configure(text="N/A")
                self.t3c1br15.configure(text=self.pendingList[self.t3_current_target-1][0].type)
                if self.pendingList[self.t3_current_target-1][0].type == "emergent":
                    self.t3c1r4.configure(state=tk.NORMAL)
                    self.t3c1r8.configure(state=tk.DISABLED)
                    self.t3c1r12.configure(state=tk.DISABLED)
                    self.t3c1r14.configure(state=tk.DISABLED)
                    self.t3c1r19.configure(state=tk.NORMAL)
                    self.t3c1b1.configure(state=tk.NORMAL)
                else:
                    self.t3c1r4.configure(state=tk.NORMAL)
                    self.t3c1r8.configure(state=tk.NORMAL)
                    self.t3c1r12.configure(state=tk.NORMAL)
                    self.t3c1r14.configure(state=tk.NORMAL)
                    self.t3c1r19.configure(state=tk.DISABLED)
                    self.t3c1b1.configure(state=tk.NORMAL)
                self.pending_bg_color = [self.pendingList[self.t3_current_target-1][0].background_color]
                self.pending_alpha_color = [self.pendingList[self.t3_current_target-1][0].alphanumeric_color]
                self.pending_orientation = [self.pendingList[self.t3_current_target-1][0].orientation]
                self.pending_description = [self.pendingList[self.t3_current_target-1][0].description]



            if pics > 1:
                query = self.interface.getCroppedImage(self.pendingList[self.t3_current_target-1][1].crop_id)
                self.t3c2i1_im = query[0]
                yaw_angle = self.getYawAngle(query[1])
                self.t3c2i1_im = self.t3c2i1_im.rotate(yaw_angle,expand=1)
                self.t3c2i1_org_width,self.t3c2i1_org_height = self.t3c2i1_im.size
                self.t3c2i1_tk = tab_tools.im2tk(self.t3c2i1_im)
                self.t3c2i1.configure(image=self.t3c2i1_tk)
                display_shape = self.pendingList[self.t3_current_target-1][1].shape
                if display_shape != None:
                    self.t3c2br5.configure(text=display_shape)
                else:
                    self.t3c2br5.configure(text="N/A")
                display_bg_color = self.pendingList[self.t3_current_target-1][1].background_color
                if display_bg_color != None:
                    self.t3c2br7.configure(text=display_bg_color)
                else:
                    self.t3c2br7.configure(text="N/A")
                display_alphanumeric = self.pendingList[self.t3_current_target-1][1].alphanumeric
                if display_alphanumeric != None:
                    self.t3c2br9.configure(text=display_alphanumeric)
                else:
                    self.t3c2br9.configure(text="N/A")
                display_alpha_color = self.pendingList[self.t3_current_target-1][1].alphanumeric_color
                if display_alpha_color != None:
                    self.t3c2br11.configure(text=display_alpha_color)
                else:
                    self.t3c2br11.configure(text="N/A")
                display_orientation = self.pendingList[self.t3_current_target-1][1].orientation
                if display_orientation != None:
                    self.t3c2br13.configure(text=display_orientation)
                else:
                    self.t3c2br13.configure(text="N/A")
                display_decription = self.pendingList[self.t3_current_target-1][1].description
                if display_decription != "":
                    self.t3c2r18.configure(text=display_decription)
                else:
                    self.t3c2r18.configure(text="N/A")
                self.t3c2br15.configure(text=self.pendingList[self.t3_current_target-1][1].type)
                if self.pendingList[self.t3_current_target-1][1].type == "emergent":
                    self.t3c2r4.configure(state=tk.NORMAL)
                    self.t3c2r8.configure(state=tk.DISABLED)
                    self.t3c2r12.configure(state=tk.DISABLED)
                    self.t3c2r14.configure(state=tk.DISABLED)
                    self.t3c2r19.configure(state=tk.NORMAL)
                    self.t3c2b1.configure(state=tk.NORMAL)
                else:
                    self.t3c2r4.configure(state=tk.NORMAL)
                    self.t3c2r8.configure(state=tk.NORMAL)
                    self.t3c2r12.configure(state=tk.NORMAL)
                    self.t3c2r14.configure(state=tk.NORMAL)
                    self.t3c2r19.configure(state=tk.DISABLED)
                    self.t3c2b1.configure(state=tk.NORMAL)
                self.pending_bg_color.append(self.pendingList[self.t3_current_target-1][1].background_color)
                self.pending_alpha_color.append(self.pendingList[self.t3_current_target-1][1].alphanumeric_color)
                self.pending_orientation.append(self.pendingList[self.t3_current_target-1][1].orientation)
                self.pending_description.append(self.pendingList[self.t3_current_target-1][1].description)

            else:
                self.t3c2i1_im = self.t3_default_im.copy()
                self.t3c2i1_org_width,self.t3c2i1_org_height = self.t3c2i1_im.size
                self.t3c2i1_tk = tab_tools.im2tk(self.t3c2i1_im)
                self.t3c2i1.configure(image=self.t3c2i1_tk)
                self.t3c2br5.configure(text="N/A")
                self.t3c2br7.configure(text="N/A")
                self.t3c2br9.configure(text="N/A")
                self.t3c2br11.configure(text="N/A")
                self.t3c2br13.configure(text="N/A")
                self.t3c2br15.configure(text="N/A")
                self.t3c2r18.configure(text="N/A")
                self.t3c2r4.configure(state=tk.DISABLED)
                self.t3c2r8.configure(state=tk.DISABLED)
                self.t3c2r12.configure(state=tk.DISABLED)
                self.t3c2r14.configure(state=tk.DISABLED)
                self.t3c2r19.configure(state=tk.DISABLED)
                self.t3c2b1.configure(state=tk.DISABLED)

            if pics > 2:
                query = self.interface.getCroppedImage(self.pendingList[self.t3_current_target-1][2].crop_id)
                self.t3c3i1_im = query[0]
                yaw_angle = self.getYawAngle(query[1])
                self.t3c3i1_im = self.t3c3i1_im.rotate(yaw_angle,expand=1)
                self.t3c3i1_org_width,self.t3c3i1_org_height = self.t3c3i1_im.size
                self.t3c3i1_tk = tab_tools.im2tk(self.t3c3i1_im)
                self.t3c3i1.configure(image=self.t3c3i1_tk)
                display_shape = self.pendingList[self.t3_current_target-1][2].shape
                if display_shape != None:
                    self.t3c3br5.configure(text=display_shape)
                else:
                    self.t3c3br5.configure(text="N/A")
                display_bg_color = self.pendingList[self.t3_current_target-1][2].background_color
                if display_bg_color != None:
                    self.t3c3br7.configure(text=display_bg_color)
                else:
                    self.t3c3br7.configure(text="N/A")
                display_alphanumeric = self.pendingList[self.t3_current_target-1][2].alphanumeric
                if display_alphanumeric != None:
                    self.t3c3br9.configure(text=display_alphanumeric)
                else:
                    self.t3c3br9.configure(text="N/A")
                display_alpha_color = self.pendingList[self.t3_current_target-1][2].alphanumeric_color
                if display_alpha_color != None:
                    self.t3c3br11.configure(text=display_alpha_color)
                else:
                    self.t3c3br11.configure(text="N/A")
                display_orientation = self.pendingList[self.t3_current_target-1][2].orientation
                if display_orientation != None:
                    self.t3c3br13.configure(text=display_orientation)
                else:
                    self.t3c3br13.configure(text="N/A")
                display_decription = self.pendingList[self.t3_current_target-1][2].description
                if display_decription != "":
                    self.t3c3r18.configure(text=display_decription)
                else:
                    self.t3c3r18.configure(text="N/A")
                self.t3c3br15.configure(text=self.pendingList[self.t3_current_target-1][2].type)
                if self.pendingList[self.t3_current_target-1][2].type == "emergent":
                    self.t3c3r4.configure(state=tk.NORMAL)
                    self.t3c3r8.configure(state=tk.DISABLED)
                    self.t3c3r12.configure(state=tk.DISABLED)
                    self.t3c3r14.configure(state=tk.DISABLED)
                    self.t3c3r19.configure(state=tk.NORMAL)
                    self.t3c3b1.configure(state=tk.NORMAL)
                else:
                    self.t3c3r4.configure(state=tk.NORMAL)
                    self.t3c3r8.configure(state=tk.NORMAL)
                    self.t3c3r12.configure(state=tk.NORMAL)
                    self.t3c3r14.configure(state=tk.NORMAL)
                    self.t3c3r19.configure(state=tk.DISABLED)
                    self.t3c3b1.configure(state=tk.NORMAL)
                self.pending_bg_color.append(self.pendingList[self.t3_current_target-1][2].background_color)
                self.pending_alpha_color.append(self.pendingList[self.t3_current_target-1][2].alphanumeric_color)
                self.pending_orientation.append(self.pendingList[self.t3_current_target-1][2].orientation)
                self.pending_description.append(self.pendingList[self.t3_current_target-1][2].description)

            else:
                self.t3c3i1_im = self.t3_default_im.copy()
                self.t3c3i1_org_width,self.t3c3i1_org_height = self.t3c3i1_im.size
                self.t3c3i1_tk = tab_tools.im2tk(self.t3c3i1_im)
                self.t3c3i1.configure(image=self.t3c3i1_tk)
                self.t3c3br5.configure(text="N/A")
                self.t3c3br7.configure(text="N/A")
                self.t3c3br9.configure(text="N/A")
                self.t3c3br11.configure(text="N/A")
                self.t3c3br13.configure(text="N/A")
                self.t3c3br15.configure(text="N/A")
                self.t3c3r18.configure(text="N/A")
                self.t3c3r4.configure(state=tk.DISABLED)
                self.t3c3r8.configure(state=tk.DISABLED)
                self.t3c3r12.configure(state=tk.DISABLED)
                self.t3c3r14.configure(state=tk.DISABLED)
                self.t3c3r19.configure(state=tk.DISABLED)
                self.t3c3b1.configure(state=tk.DISABLED)

            if pics > 3:
                query = self.interface.getCroppedImage(self.pendingList[self.t3_current_target-1][3].crop_id)
                self.t3c4i1_im = query[0]
                yaw_angle = self.getYawAngle(query[1])
                self.t3c4i1_im = self.t3c4i1_im.rotate(yaw_angle,expand=1)
                self.t3c4i1_org_width,self.t3c4i1_org_height = self.t3c4i1_im.size
                self.t3c4i1_tk = tab_tools.im2tk(self.t3c4i1_im)
                self.t3c4i1.configure(image=self.t3c4i1_tk)
                display_shape = self.pendingList[self.t3_current_target-1][3].shape
                if display_shape != None:
                    self.t3c4br5.configure(text=display_shape)
                else:
                    self.t3c4br5.configure(text="N/A")
                display_bg_color = self.pendingList[self.t3_current_target-1][3].background_color
                if display_bg_color != None:
                    self.t3c4br7.configure(text=display_bg_color)
                else:
                    self.t3c4br7.configure(text="N/A")
                display_alphanumeric = self.pendingList[self.t3_current_target-1][3].alphanumeric
                if display_alphanumeric != None:
                    self.t3c4br9.configure(text=display_alphanumeric)
                else:
                    self.t3c4br9.configure(text="N/A")
                display_alpha_color = self.pendingList[self.t3_current_target-1][3].alphanumeric_color
                if display_alpha_color != None:
                    self.t3c4br11.configure(text=display_alpha_color)
                else:
                    self.t3c4br11.configure(text="N/A")
                display_orientation = self.pendingList[self.t3_current_target-1][3].orientation
                if display_orientation != None:
                    self.t3c4br13.configure(text=display_orientation)
                else:
                    self.t3c4br13.configure(text="N/A")
                display_decription = self.pendingList[self.t3_current_target-1][3].description
                if display_decription != "":
                    self.t3c4r18.configure(text=display_decription)
                else:
                    self.t3c4r18.configure(text="N/A")
                self.t3c4br15.configure(text=self.pendingList[self.t3_current_target-1][3].type)
                if self.pendingList[self.t3_current_target-1][3].type == "emergent":
                    self.t3c4r4.configure(state=tk.NORMAL)
                    self.t3c4r8.configure(state=tk.DISABLED)
                    self.t3c4r12.configure(state=tk.DISABLED)
                    self.t3c4r14.configure(state=tk.DISABLED)
                    self.t3c4r19.configure(state=tk.NORMAL)
                    self.t3c4b1.configure(state=tk.NORMAL)
                else:
                    self.t3c4r4.configure(state=tk.NORMAL)
                    self.t3c4r8.configure(state=tk.NORMAL)
                    self.t3c4r12.configure(state=tk.NORMAL)
                    self.t3c4r14.configure(state=tk.NORMAL)
                    self.t3c4r19.configure(state=tk.DISABLED)
                    self.t3c4b1.configure(state=tk.NORMAL)
                self.pending_bg_color.append(self.pendingList[self.t3_current_target-1][3].background_color)
                self.pending_alpha_color.append(self.pendingList[self.t3_current_target-1][3].alphanumeric_color)
                self.pending_orientation.append(self.pendingList[self.t3_current_target-1][3].orientation)
                self.pending_description.append(self.pendingList[self.t3_current_target-1][3].description)

            else:
                self.t3c4i1_im = self.t3_default_im.copy()
                self.t3c4i1_org_width,self.t3c4i1_org_height = self.t3c4i1_im.size
                self.t3c4i1_tk = tab_tools.im2tk(self.t3c4i1_im)
                self.t3c4i1.configure(image=self.t3c4i1_tk)
                self.t3c4br5.configure(text="N/A")
                self.t3c4br7.configure(text="N/A")
                self.t3c4br9.configure(text="N/A")
                self.t3c4br11.configure(text="N/A")
                self.t3c4br13.configure(text="N/A")
                self.t3c4br15.configure(text="N/A")
                self.t3c4r18.configure(text="N/A")
                self.t3c4r4.configure(state=tk.DISABLED)
                self.t3c4r8.configure(state=tk.DISABLED)
                self.t3c4r12.configure(state=tk.DISABLED)
                self.t3c4r14.configure(state=tk.DISABLED)
                self.t3c4r19.configure(state=tk.DISABLED)
                self.t3c4b1.configure(state=tk.DISABLED)

            if pics > 4:
                query = self.interface.getCroppedImage(self.pendingList[self.t3_current_target-1][4].crop_id)
                self.t3c5i1_im = query[0]
                yaw_angle = self.getYawAngle(query[1])
                self.t3c5i1_im = self.t3c5i1_im.rotate(yaw_angle,expand=1)
                self.t3c5i1_org_width,self.t3c5i1_org_height = self.t3c5i1_im.size
                self.t3c5i1_tk = tab_tools.im2tk(self.t3c5i1_im)
                self.t3c5i1.configure(image=self.t3c5i1_tk)
                display_shape = self.pendingList[self.t3_current_target-1][4].shape
                if display_shape != None:
                    self.t3c5br5.configure(text=display_shape)
                else:
                    self.t3c5br5.configure(text="N/A")
                display_bg_color = self.pendingList[self.t3_current_target-1][4].background_color
                if display_bg_color != None:
                    self.t3c5br7.configure(text=display_bg_color)
                else:
                    self.t3c5br7.configure(text="N/A")
                display_alphanumeric = self.pendingList[self.t3_current_target-1][4].alphanumeric
                if display_alphanumeric != None:
                    self.t3c5br9.configure(text=display_alphanumeric)
                else:
                    self.t3c5br9.configure(text="N/A")
                display_alpha_color = self.pendingList[self.t3_current_target-1][4].alphanumeric_color
                if display_alpha_color != None:
                    self.t3c5br11.configure(text=display_alpha_color)
                else:
                    self.t3c5br11.configure(text="N/A")
                display_orientation = self.pendingList[self.t3_current_target-1][4].orientation
                if display_orientation != None:
                    self.t3c5br13.configure(text=display_orientation)
                else:
                    self.t3c5br13.configure(text="N/A")
                display_decription = self.pendingList[self.t3_current_target-1][4].description
                if display_decription != "":
                    self.t3c5r18.configure(text=display_decription)
                else:
                    self.t3c5r18.configure(text="N/A")
                self.t3c5br15.configure(text=self.pendingList[self.t3_current_target-1][4].type)
                if self.pendingList[self.t3_current_target-1][4].type == "emergent":
                    self.t3c5r4.configure(state=tk.NORMAL)
                    self.t3c5r8.configure(state=tk.DISABLED)
                    self.t3c5r12.configure(state=tk.DISABLED)
                    self.t3c5r14.configure(state=tk.DISABLED)
                    self.t3c5r19.configure(state=tk.NORMAL)
                    self.t3c5b1.configure(state=tk.NORMAL)
                else:
                    self.t3c5r4.configure(state=tk.NORMAL)
                    self.t3c5r8.configure(state=tk.NORMAL)
                    self.t3c5r12.configure(state=tk.NORMAL)
                    self.t3c5r14.configure(state=tk.NORMAL)
                    self.t3c5r19.configure(state=tk.DISABLED)
                    self.t3c5b1.configure(state=tk.NORMAL)
                self.pending_bg_color.append(self.pendingList[self.t3_current_target-1][4].background_color)
                self.pending_alpha_color.append(self.pendingList[self.t3_current_target-1][4].alphanumeric_color)
                self.pending_orientation.append(self.pendingList[self.t3_current_target-1][4].orientation)
                self.pending_description.append(self.pendingList[self.t3_current_target-1][4].description)

            else:
                self.t3c5i1_im = self.t3_default_im.copy()
                self.t3c5i1_org_width,self.t3c5i1_org_height = self.t3c5i1_im.size
                self.t3c5i1_tk = tab_tools.im2tk(self.t3c5i1_im)
                self.t3c5i1.configure(image=self.t3c5i1_tk)
                self.t3c5br5.configure(text="N/A")
                self.t3c5br7.configure(text="N/A")
                self.t3c5br9.configure(text="N/A")
                self.t3c5br11.configure(text="N/A")
                self.t3c5br13.configure(text="N/A")
                self.t3c5br15.configure(text="N/A")
                self.t3c5r18.configure(text="N/A")
                self.t3c5r4.configure(state=tk.DISABLED)
                self.t3c5r8.configure(state=tk.DISABLED)
                self.t3c5r12.configure(state=tk.DISABLED)
                self.t3c5r14.configure(state=tk.DISABLED)
                self.t3c5r19.configure(state=tk.DISABLED)
                self.t3c5b1.configure(state=tk.DISABLED)

            if self.pendingList != None:
                # Possible Submission
                self.t3c6i1_im = self.t3c1i1_im.copy()
                self.t3c6i1_org_width,self.t3c6i1_org_height = self.t3c6i1_im.size
                self.t3c6i1_tk = tab_tools.im2tk(self.t3c6i1_im)
                self.t3c6i1.configure(image=self.t3c6i1_tk)
                display_shape = self.pendingList[self.t3_current_target-1][0].shape
                if display_shape != None:
                    self.t3c6br5.configure(text=display_shape)
                else:
                    self.t3c6br5.configure(text="N/A")
                display_alphanumeric = self.pendingList[self.t3_current_target-1][0].alphanumeric
                if display_alphanumeric != None:
                    self.t3c6br9.configure(text=display_alphanumeric)
                else:
                    self.t3c6br9.configure(text="N/A")
                self.t3c6br15.configure(text=self.pendingList[self.t3_current_target-1][0].type)
                display_bg_color = self.findMostCommonValue(self.pending_bg_color)
                if display_bg_color != None:
                    self.t3c6br7.configure(text=display_bg_color)
                else:
                    self.t3c6br7.configure(text="N/A")
                display_alpha_color = self.findMostCommonValue(self.pending_alpha_color)
                if display_alpha_color != None:
                    self.t3c6br11.configure(text=display_alpha_color)
                else:
                    self.t3c6br11.configure(text="N/A")
                display_orientation = self.findMostCommonValue(self.pending_orientation)
                if display_orientation != None:
                    self.t3c6br13.configure(text=display_orientation)
                else:
                    self.t3c6br13.configure(text="N/A")
                display_description = self.findMostCommonValue(self.pending_description)
                if display_description != "":
                    self.t3c6r18.configure(text=display_description)
                else:
                    self.t3c6r18.configure(text="N/A")
                for ii in range(5):
                    self.resizeEventTab3()

        else:
            error_np = tab_tools.get_image('assets/server_error.jpg')
            error_im = tab_tools.np2im(error_np)
            self.t3c1i1_im = error_im.copy()
            self.t3c1i1_tk = tab_tools.im2tk(self.t3c1i1_im)
            self.t3c1i1_org_width,self.t3c1i1_org_height = self.t3c1i1_im.size
            self.t3c1i1.configure(image=self.t3c1i1_tk)
            self.t3c2i1_im = error_im.copy()
            self.t3c2i1_tk = tab_tools.im2tk(self.t3c2i1_im)
            self.t3c2i1_org_width,self.t3c2i1_org_height = self.t3c2i1_im.size
            self.t3c2i1.configure(image=self.t3c2i1_tk)
            self.t3c3i1_im = error_im.copy()
            self.t3c3i1_tk = tab_tools.im2tk(self.t3c3i1_im)
            self.t3c3i1_org_width,self.t3c3i1_org_height = self.t3c3i1_im.size
            self.t3c3i1.configure(image=self.t3c3i1_tk)
            self.t3c4i1_im = error_im.copy()
            self.t3c4i1_tk = tab_tools.im2tk(self.t3c4i1_im)
            self.t3c4i1_org_width,self.t3c4i1_org_height = self.t3c4i1_im.size
            self.t3c4i1.configure(image=self.t3c4i1_tk)
            self.t3c5i1_im = error_im.copy()
            self.t3c5i1_tk = tab_tools.im2tk(self.t3c5i1_im)
            self.t3c5i1_org_width,self.t3c5i1_org_height = self.t3c5i1_im.size
            self.t3c5i1.configure(image=self.t3c5i1_tk)
            self.t3c6i1_im = error_im.copy()
            self.t3c6i1_tk = tab_tools.im2tk(self.t3c6i1_im)
            self.t3c6i1_org_width,self.t3c6i1_org_height = self.t3c6i1_im.size
            self.t3c6i1.configure(image=self.t3c6i1_tk)
            self.resizeEventTab3()

    def nextClassified(self,event):
        """
        Goes to next classified target group

        @type  event: event
        @param event: Right arrow event

        @rtype:  None
        @return: None
        """
        #self.updateManualSubmissionTab()
        self.serverConnected = self.interface.ping()
        if self.serverConnected:
            if self.pendingList == None:
                self.t3_current_target = 0
            elif self.t3_current_target < len(self.pendingList):
                self.t3_current_target += 1
            else:
                pass

        #reset submission variables
        self.submit_crop_id = None
        self.submit_orientation = None
        self.submit_bg_color = None
        self.submit_alpha_color = None
        self.submit_desc = None

        self.submissionBackgroundColor.set(None)
        self.selectBackgroundColor()
        self.submissionAlphanumericColor.set(None)
        self.selectAlphanumericColor()
        self.submissionOrientation.set(None)
        self.selectOrientation()
        self.submissionDescription.set(None)
        self.selectDescription()
        self.submissionImage.set(None)
        self.selectImage()
        self.updateManualSubmissionTab()


    def previousClassified(self,event):
        """
        Goes to previous classified target group

        @type  event: event
        @param event: Right arrow event

        @rtype:  None
        @return: None
        """
        #self.updateManualSubmissionTab()
        self.serverConnected = self.interface.ping()
        if self.serverConnected:
            if self.t3_current_target > 1:
                self.t3_current_target -= 1
            elif self.pendingList == None:
                self.t3_current_target = 0
            else:
                pass

        #reset submission variables
        self.submit_crop_id = None
        self.submit_orientation = None
        self.submit_bg_color = None
        self.submit_alpha_color = None
        self.submit_desc = None

        self.submissionBackgroundColor.set(None)
        self.selectBackgroundColor()
        self.submissionAlphanumericColor.set(None)
        self.selectAlphanumericColor()
        self.submissionOrientation.set(None)
        self.selectOrientation()
        self.submissionDescription.set(None)
        self.selectDescription()
        self.submissionImage.set(None)
        self.selectImage()
        self.updateManualSubmissionTab()


    def submitTarget(self,event=None):
        """
        Submits Target to server

        @type  event: event
        @param event: Enter press or button press event

        @rtype:  None
        @return: None
        """
        submission = client_rest.TargetSubmission(self.submit_crop_id,self.submit_orientation,self.submit_bg_color,self.submit_alpha_color,self.submit_desc)
        self.interface.postSubmitTargetById(self.target_id,submission)
        if self.t3_current_target == self.t3_total_targets:
            self.t3_current_target -= 1
        self.submissionBackgroundColor.set(None)
        self.selectBackgroundColor()
        self.submissionAlphanumericColor.set(None)
        self.selectAlphanumericColor()
        self.submissionOrientation.set(None)
        self.selectOrientation()
        self.submissionDescription.set(None)
        self.selectDescription()
        self.submissionImage.set(None)
        self.selectImage()
        self.updateManualSubmissionTab()

    def findMostCommonValue(self, classifications):
        """
        Calculate the most common value in a specified column (useful for all the enum columns)

        @type classifications: list of value lists (ie: from a cursor.fetchall())
        @param classifications: database rows to use to calculate the average
        @type clmnNun: int
        @param clmnNum: Integer of the column to access in each row to get values for avg calculation
        """

        # dictionary keeps track of how many times we've seen a particular column value
        # EX: {
        #       "red": 2,
        #        "white": 1}
        valueCounts = {}
        # for each classification in our list of classifications
        # a classification here is a list
        for classification in classifications:
            if classification is not None:
                # if the value at the classification has not been added to our dictionary yet
                if classification not in valueCounts:
                    valueCounts[classification] = 0
                valueCounts[classification] += 1 # increment the particular value in the dict

        if valueCounts: # if the dictionary isnt empty
            mostCommon = max(valueCounts, key=valueCounts.get)
            return mostCommon
        return None # if there are no values for this particular field, return None

    def getYawAngle(self,imageID):
        info = self.interface.getCroppedImageInfo(imageID)
        image_state = None
        if info is not None:
            image_state = self.interface.getStateByTs(info.time_stamp)
        if image_state == None:
            #yaw_angle = 0.0
            # todo: eventually remove this
            yaw = np.random.uniform(0,360)
        else:
            yaw = image_state.yaw
        return(yaw)

    def selectBackgroundColor(self):
        try:
            value = self.submissionBackgroundColor.get()
        except Exception:
            value = None
        self.t3c1ar7.configure(foreground='#636363')#,font=('normal','11'))
        self.t3c1br7.configure(foreground='#636363')
        self.t3c2ar7.configure(foreground='#636363')
        self.t3c2br7.configure(foreground='#636363')
        self.t3c3ar7.configure(foreground='#636363')
        self.t3c3br7.configure(foreground='#636363')
        self.t3c4ar7.configure(foreground='#636363')
        self.t3c4br7.configure(foreground='#636363')
        self.t3c5ar7.configure(foreground='#636363')
        self.t3c5br7.configure(foreground='#636363')
        self.t3c6ar7.configure(foreground='#636363')
        self.t3c6br7.configure(foreground='#636363')

        if value == None:
            pass
        else:
            self.t3c6ar7.configure(foreground='blue')
            self.t3c6br7.configure(foreground='blue')
            if value == 1:
                self.t3c1br7.configure(foreground='blue')
                self.t3c1ar7.configure(foreground='blue')
            elif value == 2:
                self.t3c2ar7.configure(foreground='blue')
                self.t3c2br7.configure(foreground='blue')
            elif value == 3:
                self.t3c3ar7.configure(foreground='blue')
                self.t3c3br7.configure(foreground='blue')
            elif value == 4:
                self.t3c4ar7.configure(foreground='blue')
                self.t3c4br7.configure(foreground='blue')
            else:
                self.t3c5ar7.configure(foreground='blue')
                self.t3c5br7.configure(foreground='blue')
            self.t3c6br7.configure(text=self.pendingList[self.t3_current_target-1][value-1].background_color)
            self.submit_bg_color = self.pendingList[self.t3_current_target-1][value-1].class_id

    def selectAlphanumericColor(self):
        try:
            value = self.submissionAlphanumericColor.get()
        except Exception:
            value = None
        self.t3c1ar11.configure(foreground='#636363')#,font=('normal','11'))
        self.t3c1br11.configure(foreground='#636363')
        self.t3c2ar11.configure(foreground='#636363')
        self.t3c2br11.configure(foreground='#636363')
        self.t3c3ar11.configure(foreground='#636363')
        self.t3c3br11.configure(foreground='#636363')
        self.t3c4ar11.configure(foreground='#636363')
        self.t3c4br11.configure(foreground='#636363')
        self.t3c5ar11.configure(foreground='#636363')
        self.t3c5br11.configure(foreground='#636363')
        self.t3c6ar11.configure(foreground='#636363')
        self.t3c6br11.configure(foreground='#636363')

        if value == None:
            pass
        else:
            self.t3c6ar11.configure(foreground='blue')
            self.t3c6br11.configure(foreground='blue')
            if value == 1:
                self.t3c1br11.configure(foreground='blue')
                self.t3c1ar11.configure(foreground='blue')
            elif value == 2:
                self.t3c2ar11.configure(foreground='blue')
                self.t3c2br11.configure(foreground='blue')
            elif value == 3:
                self.t3c3ar11.configure(foreground='blue')
                self.t3c3br11.configure(foreground='blue')
            elif value == 4:
                self.t3c4ar11.configure(foreground='blue')
                self.t3c4br11.configure(foreground='blue')
            else:
                self.t3c5ar11.configure(foreground='blue')
                self.t3c5br11.configure(foreground='blue')
            self.t3c6br11.configure(text=self.pendingList[self.t3_current_target-1][value-1].alphanumeric_color)
            self.submit_alpha_color = self.pendingList[self.t3_current_target-1][value-1].class_id

    def selectOrientation(self):
        try:
            value = self.submissionOrientation.get()
        except Exception:
            value = None

        self.t3c1ar13.configure(foreground='#636363')#,font=('normal','11'))
        self.t3c1br13.configure(foreground='#636363')
        self.t3c2ar13.configure(foreground='#636363')
        self.t3c2br13.configure(foreground='#636363')
        self.t3c3ar13.configure(foreground='#636363')
        self.t3c3br13.configure(foreground='#636363')
        self.t3c4ar13.configure(foreground='#636363')
        self.t3c4br13.configure(foreground='#636363')
        self.t3c5ar13.configure(foreground='#636363')
        self.t3c5br13.configure(foreground='#636363')
        self.t3c6ar13.configure(foreground='#636363')
        self.t3c6br13.configure(foreground='#636363')

        if value == None:
            pass
        else:
            self.t3c6ar13.configure(foreground='blue')
            self.t3c6br13.configure(foreground='blue')
            if value == 1:
                self.t3c1br13.configure(foreground='blue')
                self.t3c1ar13.configure(foreground='blue')
            elif value == 2:
                self.t3c2ar13.configure(foreground='blue')
                self.t3c2br13.configure(foreground='blue')
            elif value == 3:
                self.t3c3ar13.configure(foreground='blue')
                self.t3c3br13.configure(foreground='blue')
            elif value == 4:
                self.t3c4ar13.configure(foreground='blue')
                self.t3c4br13.configure(foreground='blue')
            else:
                self.t3c5ar13.configure(foreground='blue')
                self.t3c5br13.configure(foreground='blue')
            self.t3c6br13.configure(text=self.pendingList[self.t3_current_target-1][value-1].orientation)
            self.submit_orientation = self.pendingList[self.t3_current_target-1][value-1].class_id

    def selectDescription(self):
        try:
            value = self.submissionDescription.get()
        except Exception:
            value = None
        self.t3c1r17.configure(foreground='#636363')#,font=('normal','11'))
        self.t3c1r18.configure(foreground='#636363')
        self.t3c2r17.configure(foreground='#636363')
        self.t3c2r18.configure(foreground='#636363')
        self.t3c3r17.configure(foreground='#636363')
        self.t3c3r18.configure(foreground='#636363')
        self.t3c4r17.configure(foreground='#636363')
        self.t3c4r18.configure(foreground='#636363')
        self.t3c5r17.configure(foreground='#636363')
        self.t3c5r18.configure(foreground='#636363')
        self.t3c6r17.configure(foreground='#636363')
        self.t3c6r18.configure(foreground='#636363')

        if value == None:
            pass
        else:
            self.t3c6r17.configure(foreground='blue')
            self.t3c6r18.configure(foreground='blue')
            if value == 1:
                self.t3c1r18.configure(foreground='blue')
                self.t3c1r17.configure(foreground='blue')
            elif value == 2:
                self.t3c2r17.configure(foreground='blue')
                self.t3c2r18.configure(foreground='blue')
            elif value == 3:
                self.t3c3r17.configure(foreground='blue')
                self.t3c3r18.configure(foreground='blue')
            elif value == 4:
                self.t3c4r17.configure(foreground='blue')
                self.t3c4r18.configure(foreground='blue')
            else:
                self.t3c5r17.configure(foreground='blue')
                self.t3c5r18.configure(foreground='blue')
            self.t3c6r18.configure(text=self.pendingList[self.t3_current_target-1][value-1].description)
            self.submit_desc = self.pendingList[self.t3_current_target-1][value-1].class_id

    def selectImage(self):
        try:
            value = self.submissionImage.get()
        except Exception:
            value = None

        if value == None:
            pass
        else:
            if value == 1:
                self.t3c6i1_im = self.t3c1i1_im.copy()
            elif value == 2:
                self.t3c6i1_im = self.t3c2i1_im.copy()
            elif value == 3:
                self.t3c6i1_im = self.t3c3i1_im.copy()
            elif value == 4:
                self.t3c6i1_im = self.t3c4i1_im.copy()
            else:
                self.t3c6i1_im = self.t3c5i1_im.copy()
            self.t3c6i1_org_width,self.t3c6i1_org_height = self.t3c6i1_im.size
            self.t3c6i1_tk = tab_tools.im2tk(self.t3c6i1_im)
            self.t3c6i1.configure(image=self.t3c6i1_tk)
            self.submit_crop_id = self.pendingList[self.t3_current_target-1][value-1].class_id

    def deleteClassification1(self):
        classification_id = self.pendingList[self.t3_current_target-1][0].class_id
        self.interface.deleteClass(classification_id)

        #reset submission variables if they were the deleted classification
        class_id = self.pendingList[self.t3_current_target-1][0].class_id
        if self.submit_crop_id == class_id:
            self.submissionImage.set(None)
            self.selectImage()
        if self.submit_orientation == class_id:
            self.submissionOrientation.set(None)
            self.selectOrientation()
        if self.submit_bg_color == class_id:
            self.submissionBackgroundColor.set(None)
            self.selectBackgroundColor()
        if self.submit_alpha_color == class_id:
            self.submissionAlphanumericColor.set(None)
            self.selectAlphanumericColor()
        if self.submit_desc == class_id:
            self.submissionDescription.set(None)
            self.selectDescription()

        self.updateManualSubmissionTab()

    def deleteClassification2(self):
        classification_id = self.pendingList[self.t3_current_target-1][1].class_id
        self.interface.deleteClass(classification_id)

        #reset submission variables if they were the deleted classification
        class_id = self.pendingList[self.t3_current_target-1][1].class_id
        if self.submit_crop_id == class_id:
            self.submissionImage.set(None)
            self.selectImage()
        if self.submit_orientation == class_id:
            self.submissionOrientation.set(None)
            self.selectOrientation()
        if self.submit_bg_color == class_id:
            self.submissionBackgroundColor.set(None)
            self.selectBackgroundColor()
        if self.submit_alpha_color == class_id:
            self.submissionAlphanumericColor.set(None)
            self.selectAlphanumericColor()
        if self.submit_desc == class_id:
            self.submissionDescription.set(None)
            self.selectDescription()

        self.updateManualSubmissionTab()

    def deleteClassification3(self):
        classification_id = self.pendingList[self.t3_current_target-1][2].class_id
        self.interface.deleteClass(classification_id)

        #reset submission variables if they were the deleted classification
        class_id = self.pendingList[self.t3_current_target-1][2].class_id
        if self.submit_crop_id == class_id:
            self.submissionImage.set(None)
            self.selectImage()
        if self.submit_orientation == class_id:
            self.submissionOrientation.set(None)
            self.selectOrientation()
        if self.submit_bg_color == class_id:
            self.submissionBackgroundColor.set(None)
            self.selectBackgroundColor()
        if self.submit_alpha_color == class_id:
            self.submissionAlphanumericColor.set(None)
            self.selectAlphanumericColor()
        if self.submit_desc == class_id:
            self.submissionDescription.set(None)
            self.selectDescription()

        self.updateManualSubmissionTab()

    def deleteClassification4(self):
        classification_id = self.pendingList[self.t3_current_target-1][3].class_id
        self.interface.deleteClass(classification_id)

        #reset submission variables if they were the deleted classification
        class_id = self.pendingList[self.t3_current_target-1][3].class_id
        if self.submit_crop_id == class_id:
            self.submissionImage.set(None)
            self.selectImage()
        if self.submit_orientation == class_id:
            self.submissionOrientation.set(None)
            self.selectOrientation()
        if self.submit_bg_color == class_id:
            self.submissionBackgroundColor.set(None)
            self.selectBackgroundColor()
        if self.submit_alpha_color == class_id:
            self.submissionAlphanumericColor.set(None)
            self.selectAlphanumericColor()
        if self.submit_desc == class_id:
            self.submissionDescription.set(None)
            self.selectDescription()

        self.updateManualSubmissionTab()

    def deleteClassification5(self):
        classification_id = self.pendingList[self.t3_current_target-1][4].class_id
        self.interface.deleteClass(classification_id)

        #reset submission variables if they were the deleted classification
        class_id = self.pendingList[self.t3_current_target-1][4].class_id
        if self.submit_crop_id == class_id:
            self.submissionImage.set(None)
            self.selectImage()
        if self.submit_orientation == class_id:
            self.submissionOrientation.set(None)
            self.selectOrientation()
        if self.submit_bg_color == class_id:
            self.submissionBackgroundColor.set(None)
            self.selectBackgroundColor()
        if self.submit_alpha_color == class_id:
            self.submissionAlphanumericColor.set(None)
            self.selectAlphanumericColor()
        if self.submit_desc == class_id:
            self.submissionDescription.set(None)
            self.selectDescription()

        self.updateManualSubmissionTab()

    def entry_focus_in(self,event=None):
        self.t2_entry_focus = True

    def entry_focus_out(self,event=None):
        self.t2_entry_focus = False
        # set focus onto image (away from entry widget)
        self.t2c2i1.focus_set()

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
