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

class GuiClass():
    """This is the classification Gui"""
    def __init__(self):
        self.root = Tk()
        self.root.title("Tab Example")
        self.root.attributes('-zoomed', True) # maximizes screen
        n = ttk.Notebook(self.root)
        n.pack(fill=BOTH, padx=6,pady=9)
        f1 = ttk.Frame(n)   # first page, which would get widgets gridded into it
        f2 = ttk.Frame(n)   # second page
        f3 = ttk.Frame(n)
        n.add(f1, text='Cropping')
        n.add(f2, text='Classification')
        n.add(f3, text='Autonomous Tender')

    def main(self):
        self.root.mainloop()



if __name__ == "__main__":
    example = GuiClass()
    example.main()
