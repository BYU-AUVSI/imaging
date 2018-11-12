'''
Author: D. Knowles
Date: 11/6/18
Description: Testing out TK gui options

Prereqs:
python 3
sudo apt install python3-tk
pip3 install Pillow, opencv-python
'''
from tkinter import *
from tkinter import ttk
from PIL import Image,ImageTk
import cv2

class GuiClass(Frame):
    """Classification Gui for AUVSi 2019"""
    def __init__(self,master=None):
        Frame.__init__(self,master=None)
        self.master = master # gui master handle
        self.master.title("AUVSI COMPETITION 2019: CLASSIFICATION")
        self.master.attributes('-zoomed', True) # maximizes screen
        n = ttk.Notebook(self.master) # create tabs
        n.pack(fill=BOTH, expand=1) # expand across space
        self.master.bind("<Escape>",self.close_window) # press ESC to exit
        self.master.bind("<Configure>",self.resizeEvent)

        # itialize variables
        self.target_number = 0
        self.get_image()




        # TAB 1: CROPPING
        tab1 = ttk.Frame(n)   # first page, which would get widgets gridded into it
        n.add(tab1, text='Cropping')
        Grid.rowconfigure(self.master,0,weight=1)
        Grid.columnconfigure(self.master,0,weight=1)
        grid=Frame(tab1)
        grid.grid(sticky=N+S+E+W,column=0,row=7,columnspan=2)
        Grid.rowconfigure(tab1,0,weight=1)
        Grid.columnconfigure(tab1,0,weight=1)
        for x in range(19):
            Grid.columnconfigure(tab1,x,weight=1)
        for y in range(13):
            Grid.rowconfigure(tab1,y,weight=1)
        self.lbl2 = Label(tab1, text=self.target_number)
        self.lbl2.grid(row=12,column=0,sticky=N+S+E+W,padx=5,pady=5,ipadx=5,ipady=5)
        but1 = Button(tab1, text="Advance Target",command=self.bt1_clicked)
        but1.grid(row=12,column=1,sticky=N+S+E+W,padx=5,pady=5,ipadx=5,ipady=5)
        self.lbl3 = Label(tab1,image=self.image)
        self.lbl3.image = self.image
        self.lbl3.grid(row=0,column=0,rowspan=12,columnspan=12,sticky=N+S+E+W,padx=5,pady=5,ipadx=5,ipady=5)
        self.lbl4 = Label(tab1,text="initial")
        self.lbl4.grid(row=12,column=2,sticky=N+S+E+W,padx=5,pady=5,ipadx=5,ipady=5)
        self.lbl3.bind("<Button-1>",self.mouse_click)
        #tab1.bind("<space>",self.get_image())
        self.master.update()
        self.lbl10 = Label(tab1,text=self.lbl3.winfo_height())
        self.lbl10.grid(row=12,column=3,sticky=N+S+E+W,padx=5,pady=5,ipadx=5,ipady=5)
        self.lbl1 = Label(tab1, text='Target Number')
        self.lbl1.grid(row=0,column=13,sticky=N+S+E+W,padx=5,pady=5,ipadx=5,ipady=5)
        self.lbl5 = Label(tab1, text='Target Pic')
        self.lbl5.grid(row=0,column=14,sticky=N+S+E+W,padx=5,pady=5,ipadx=5,ipady=5)
        self.lbl6 = Label(tab1, text='Pic Quantity')
        self.lbl6.grid(row=0,column=15,sticky=N+S+E+W,padx=5,pady=5,ipadx=5,ipady=5)
        self.lbl7 = Label(tab1, text='Target Number')
        self.lbl7.grid(row=0,column=16,sticky=N+S+E+W,padx=5,pady=5,ipadx=5,ipady=5)
        self.lbl8 = Label(tab1, text='Target Pic')
        self.lbl8.grid(row=0,column=17,sticky=N+S+E+W,padx=5,pady=5,ipadx=5,ipady=5)
        self.lbl9 = Label(tab1, text='Pic Quantity')
        self.lbl9.grid(row=0,column=18,sticky=N+S+E+W,padx=5,pady=5,ipadx=5,ipady=5)

        for ii in range(12):
            new_label1 = Label(tab1,text=ii+1)
            new_label1.grid(row=ii+1,column=13,sticky=N+S+E+W,padx=5,pady=5,ipadx=5,ipady=5)
            new_label1 = Label(tab1,text=ii+13)
            new_label1.grid(row=ii+1,column=16,sticky=N+S+E+W,padx=5,pady=5,ipadx=5,ipady=5)



        # TAB 2: CLASSIFICATION
        tab2 = ttk.Frame(n)   # second page
        n.add(tab2, text='Classification')

        # TAB 3: AUTONOMOUS TENDER
        tab3 = ttk.Frame(n)
        n.add(tab3, text='Autonomous Tender')


        '''
        def opencv_click(event,x,y,flags,param):
            if event == cv2.EVENT_LBUTTONDOWN:
                print('Hello!')
                self.lbl4.configure(text=(x,y))
        cv2.namedWindow('tab1')
        cv2.setMouseCallback('tab1',opencv_click)
        '''
    def get_image(self):
        self.image_np = cv2.imread('example.jpg')
        self.image_np = cv2.cvtColor(self.image_np,cv2.COLOR_BGR2RGB)
        self.image = self.img_np2tk(self.image_np)
        print("ran this")
    def img_np2tk(self,image):
        new_image = Image.fromarray(image)
        new_image = ImageTk.PhotoImage(new_image)
        return new_image
    def bt1_clicked(self):
        self.target_number += 1
        self.lbl2.configure(text=self.target_number)
    def mouse_click(self,event):
        self.lbl4.configure(text=(event.x,event.y))
        self.lbl3.bind("<Motion>",self.mouse_move)
        self.x0 = event.x
        self.y0 = event.y
    def mouse_move(self,event):
        self.lbl4.configure(text=(event.x,event.y))
        self.lbl3.bind("<ButtonRelease-1>",self.mouse_release)
    def close_window(self,event):
        self.master.withdraw()
        sys.exit()
    def mouse_release(self,event):
        self.lbl3.unbind("<Motion>")
        self.lbl4.configure(text=(event.x,event.y))
        self.lbl3.unbind("<ButtonRelease-1")
        cv2.rectangle(self.image_np,(self.x0,self.y0),(event.x,event.y),(0,255,0),2)
        self.image = self.img_np2tk(self.image_np)
        self.lbl3.configure(image=self.image)
    def resizeEvent(self,event):
        print("Resized")
        self.master.update()
        self.lbl10.configure(text=self.lbl3.winfo_height())









if __name__ == "__main__":
    root = Tk()
    example = GuiClass(root)
    example.mainloop()
