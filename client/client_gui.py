'''
Author: D. Knowles
Date: 11/6/18
Description: Testing out TK gui options

Prereqs:
python 3
sudo apt install python3-tk
pip3 install Pillow, opencv-python
'''

'''
TODO:
Add keyinterrupt "Ctrl-c" shutting down the tkinter window
Add Enter key "submitting" the cropped image
Add zooming feature
Add preview of cropped image
Add key bindings for moving targets numbers
Add sample pic of targets
Add quantity of each target pictures
'''
from tkinter import *
from tkinter import ttk
from PIL import Image,ImageTk
import cv2
import numpy as np
import time



class GuiClass(Frame):
    """Classification Gui for AUVSi 2019"""
    def __init__(self,master=None):
        Frame.__init__(self,master=None)
        self.master = master # gui master handle
        self.master.attributes('-zoomed', True) # maximizes screen
        self.master.title("AUVSI COMPETITION 2019: CLASSIFICATION")

        n = ttk.Notebook(self.master) # create tabs
        n.pack(fill=BOTH, expand=1) # expand across space
        self.master.bind("<Escape>",self.close_window) # press ESC to exit
        self.master.bind("<Configure>",self.resizeEvent)


        # itialize variables
        self.initialized = False
        self.target_number = 0
        self.org_np = self.get_image('frame0744.jpg')
        self.draw_np = np.copy(self.org_np)
        self.img_im = self.np2im(self.draw_np)
        self.crop_im = self.img_im.copy()
        self.crop_tk = self.im2tk(self.crop_im)
        self.img_tk = self.im2tk(self.img_im)
        self.org_width,self.org_height = self.img_im.size
        self.resize_counter = time.time()
        self.cropped = False




        # TAB 1: CROPPING
        tab1 = ttk.Frame(n)   # first page, which would get widgets gridded into it
        self.master.bind("<Control-z>",self.undoCrop)
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
        self.lbl3 = Label(tab1,image=self.img_tk)
        self.lbl3.image = self.img_tk
        self.lbl3.grid(row=0,column=0,rowspan=12,columnspan=12,sticky=N+S+E+W,padx=5,pady=5,ipadx=5,ipady=5)
        self.lbl3.bind("<Button-1>",self.mouse_click)
        self.lbl4 = Label(tab1,text="initial")
        self.lbl4.grid(row=12,column=2,sticky=N+S+E+W,padx=5,pady=5,ipadx=5,ipady=5)
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
        self.t1l1 = Label(tab2,image=self.crop_tk)
        self.t1l1.image = self.crop_tk
        self.t1l1.grid(row=0,column=0,rowspan=12,columnspan=12,sticky=N+S+E+W,padx=5,pady=5,ipadx=5,ipady=5)


        # TAB 3: AUTONOMOUS TENDER
        tab3 = ttk.Frame(n)
        n.add(tab3, text='Autonomous Tender')
        # Done with initialization
        self.initialized = True


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
        self.resized_im = self.resizeIm(self.img_im,self.t1l1_width,self.t1l1_height)
        self.img_tk = self.im2tk(self.resized_im)
        self.lbl3.configure(image=self.img_tk)
    def close_window(self,event):
        self.master.withdraw()
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
        self.resized_im = self.resizeIm(self.img_im,self.t1l1_width,self.t1l1_height)
        self.img_tk = self.im2tk(self.resized_im)
        self.lbl3.configure(image=self.img_tk)
        # Crop Image
        self.cropImage(int(sr*self.x0),int(sr*self.y0),int(sr*self.x1),int(sr*self.y1))
        self.cropped = True






    def resizeEvent(self,event):
        if self.initialized == True and (time.time()-self.resize_counter) > 0.050:
            if self.lbl3.winfo_width() > 1:
                self.resize_counter = time.time()
                self.master.update()
                self.t1l1_width = self.lbl3.winfo_width()
                self.t1l1_height = self.lbl3.winfo_height()
                self.resized_im = self.resizeIm(self.img_im,self.t1l1_width,self.t1l1_height)
                self.t1i1_width,self.t1i1_height = self.resized_im.size
                self.img_tk = self.im2tk(self.resized_im)
                self.lbl3.configure(image=self.img_tk)

    def resizeIm(self,image,width_restrict,height_restrict):
        ratio_h = height_restrict/self.org_height
        ratio_w = width_restrict/self.org_width
        if ratio_h <= ratio_w:
            resized_im = self.img_im.resize((int(self.org_width*ratio_h), int(self.org_height*ratio_h)), Image.ANTIALIAS)
        else:
            resized_im = self.img_im.resize((int(self.org_width*ratio_w), int(self.org_height*ratio_w)), Image.ANTIALIAS)
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
        self.crop_tk = self.im2tk(self.crop_im)
        self.t1l1.configure(image=self.crop_tk)
    def undoCrop(self,event=None):
        self.draw_np = np.copy(self.org_np)
        self.img_im = self.np2im(self.draw_np)
        self.resized_im = self.resizeIm(self.img_im,self.t1l1_width,self.t1l1_height)
        self.img_tk = self.im2tk(self.resized_im)
        self.lbl3.configure(image=self.img_tk)
        self.crop_im = self.img_im.copy()
        self.crop_tk = self.im2tk(self.crop_im)
        self.t1l1.configure(image=self.crop_tk)
    def nextImage(self,event):
        pass











if __name__ == "__main__":
    root = Tk()
    example = GuiClass(root)
    example.mainloop()
