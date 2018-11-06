'''
Author: D. Knowles
Date: 11/6/18
Description: Testing out TK gui options
'''
n = ttk.Notebook(parent)
f1 = ttk.Frame(n)   # first page, which would get widgets gridded into it
f2 = ttk.Frame(n)   # second page
n.add(f1, text='One')
n.add(f2, text='Two')