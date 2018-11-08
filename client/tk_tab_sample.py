'''
Author: D. Knowles
Date: 11/6/18
Description: Testing out TK gui options

Prereqs:
sudo apt install python3-tk
pip3 install Pillow
'''
from tkinter import *
from tkinter import ttk
from PIL import Image,ImageTk
import cv2

class GuiClass(Frame):
    """This is the classification Gui"""
    def __init__(self,master=None):
        Frame.__init__(self,master=None)
        self.master = master
        self.master.title("Tab Example")
        #self.master.attributes('-zoomed', True) # maximizes screen
        self.target_number = 0


        n = ttk.Notebook(self.master)
        n.pack(fill=BOTH, expand=1)
        tab1 = ttk.Frame(n)   # first page, which would get widgets gridded into it
        n.add(tab1, text='Cropping')
        self.lbl1 = Label(tab1, text='Target Number')
        self.lbl1.grid(row=1,column=1,sticky='E',padx=5,pady=5,ipadx=5,ipady=5)
        self.lbl2 = Label(tab1, text=self.target_number)
        self.lbl2.grid(row=1,column=2,sticky='E',padx=5,pady=5,ipadx=5,ipady=5)
        but1 = Button(tab1, text="Advance Target",command=self.clicked)
        but1.grid(row=2,column=1,sticky='E',padx=5,pady=5,ipadx=5,ipady=5)
        image = cv2.imread('example.jpg')
        image = cv2.cvtColor(image,cv2.COLOR_BGR2RGB)
        image = Image.fromarray(image)
        image = ImageTk.PhotoImage(image)
        self.lbl3 = Label(tab1,image=image)
        self.lbl3.image = image
        self.lbl3.grid(row=3,column=3,sticky='E',padx=5,pady=5,ipadx=5,ipady=5)


        tab2 = ttk.Frame(n)   # second page
        n.add(tab2, text='Classification')
        tab3 = ttk.Frame(n)
        n.add(tab3, text='Autonomous Tender')
    def clicked(self):
        self.target_number += 1
        self.lbl2.configure(text=self.target_number)









if __name__ == "__main__":
    root = Tk()
    example = GuiClass(root)
    example.mainloop()
