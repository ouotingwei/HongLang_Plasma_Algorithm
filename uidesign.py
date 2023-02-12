import tkinter as tk
root = tk.Tk()
root.title('cuteluluWindow')
root.configure(bg="#7AFEC6")
text=tk.Label(root, text="Hello\@^0^@/",font=("Bauhaus 93",20,"bold"))

count=0
def clickHello():
    global count
    count=count + 1
    text.config(text="Click Hello " + str(count) + " times")
B=tk.Button(root, text="Hello", command=clickHello,font=("Bauhaus 93",20,"bold"))

text.pack()
B.pack()

root.mainloop()