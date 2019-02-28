'''
Authors: D. Knowles, B. McBride, T. Miller
'''

'''
TODO:
    possible threading behind the scenese to autosize other tabs
    use consistent naming patterns (camel case, snake case)
    use consistent commenting pattern and placement
    change gui into multiple tabs/classes
    change text font, size, color, etc.
    organize functions into a logical order
    standardize import order

Tab4:
    create everything

KNOWN BUGS:
    When you click on the 2nd tab right at the beginning, and then use the left/right
        buttons, it moves one tab, then unbinds like it's supposed to.
'''

import tkinter as tk
from tkinter import ttk, font
from ttkthemes import ThemedStyle
from PIL import Image, ImageTk
import PIL
import cv2
import numpy as np
import time
import sys
from lib import tab0, tab1, tab2, tab3, tab4, tab_tools, client_rest


class GuiClass(tk.Frame):
    """
    Graphical User Interface for 2019 AUVSI competition

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


        # -----------------------  Tab 0: SETTINGS  ----------------------------
        self.tab0 = tab0.Tab0(self.master,self.n)
        self.interface = self.tab0.interfaceCall()

        # -----------------------  Tab 1: CROPPPING  ---------------------------
        self.tab1 = tab1.Tab1(self.master,self.n,self.interface)

        # ---------------------  Tab 2: CLASSIFICATION  ------------------------
        self.tab2 = tab2.Tab2(self.master,self.n,self.interface)

        # -------------------  Tab 3: TARGET SUBMISSION  -----------------------
        self.tab3 = tab3.Tab3(self.master,self.n,self.interface)

        # --------------  Tab 4: AUTONOMOUS TARGET SUBMISSION  -----------------
        self.tab4 = tab4.Tab4(self.master,self.n,self.interface)


        # ----------------------------KEY BINDINGS -----------------------------
        self.master.bind("<<NotebookTabChanged>>",self.tabChanged)
        self.master.bind("<Escape>",self.close_window)

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
            self.tab2.run(self.interface)
        else:
            self.tab2.stoprun()

        if active_tab == 3:
            self.tab3.run(self.interface)

        if active_tab == 4:
            self.tab4.run(self.interface)

        self.master.focus_set()

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
