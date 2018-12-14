import tkinter as tk
from tkinter.constants import *
r=tk.Tk()
from PIL import Image,ImageTk
import cv2

def set_aspect(content_frame, pad_frame, aspect_ratio):
    # a function which places a frame within a containing frame, and
    # then forces the inner frame to keep a specific aspect ratio

    def enforce_aspect_ratio(event):
        # when the pad window resizes, fit the content into it,
        # either by fixing the width or the height and then
        # adjusting the height or width based on the aspect ratio.

        # start by using the width as the controlling dimension
        desired_width = event.width
        desired_height = int(event.width / aspect_ratio)

        # if the window is too tall to fit, use the height as
        # the controlling dimension
        if desired_height > event.height:
            desired_height = event.height
            desired_width = int(event.height * aspect_ratio)

        # place the window, giving it an explicit size
        content_frame.place(in_=pad_frame, x=0, y=0,
            width=desired_width, height=desired_height)

    pad_frame.bind("<Configure>", enforce_aspect_ratio)

image = cv2.imread('example.jpg')
image = cv2.cvtColor(image,cv2.COLOR_BGR2RGB)
image = Image.fromarray(image)
image = ImageTk.PhotoImage(image)

pad_frame = tk.Frame(borderwidth=0, background="bisque", width=200, height=200)
pad_frame.grid(row=0, column=0, sticky="nsew", padx=10, pady=20)
content_frame=tk.Label(r,borderwidth=5,image=image)
content_frame2=tk.Label(r,borderwidth=5,image=image)
tk.Label(content_frame,text='wtf').pack()
set_aspect(content_frame, pad_frame, aspect_ratio=5.0/1.0)
set_aspect(content_frame2, pad_frame, aspect_ratio=5.0/1.0)
r.rowconfigure(0, weight=1)
r.columnconfigure(0, weight=1)
r.mainloop()
