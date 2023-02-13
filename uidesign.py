import tkinter as tk

global model

def clickModel_1():
    model = 1
    return model
def clickModel_2():
    model = 2
    return model
def clickModel_3():
    model = 3
    return model
def clickModel_4():
    model = 4
    return model

def main():
    root = tk.Tk()
    root.title('HL Plasma Path Planning ')
    root.configure(bg="gray")
    root.geometry("1000x500")

    description = """選擇一種模型"""

    text=tk.Label(root, text=description,bg="gray",
              compound="left", fg="black",
              font=("Viner Hand ITC", 20, "bold", "italic"))
    
    B1=tk.Button(root,text='001',relief="ridge",
            activebackground='white',
            activeforeground='#FFFFFF',
             state=tk.NORMAL, command = clickModel_1)
    B2=tk.Button(root,text='002',relief="ridge",
                activebackground='white',
                activeforeground='#FFFFFF',
                state=tk.NORMAL, command = clickModel_2)
    B3=tk.Button(root,text='003',relief="ridge",
                activebackground='white',
                activeforeground='#FFFFFF',
                state=tk.ACTIVE, command = clickModel_3)
    B4=tk.Button(root,text='004',relief="ridge",
                activebackground='white',
                activeforeground='#FFFFFF',
                state=tk.ACTIVE, command = clickModel_4)

    text.pack()
    B1.pack()
    B2.pack()
    B3.pack()
    B4.pack()

    root.mainloop()


if __name__ == '__main__':
    main()