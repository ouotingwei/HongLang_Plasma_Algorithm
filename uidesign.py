import tkinter as tk
root = tk.Tk()
root.title('cuteluluWindow')
root.configure(bg="#7AFEC6")
text=tk.Label(root, text="Hello\@^0^@/",font=("Bauhaus 93",20,"bold"))

count=0
def clickModel():
    global model
    model = 0
    count=count + 1
    text.config(text="Click Hello " + str(count) + " times")

    if model != 0:
        return True
    
    else:
        return False


text.pack()
B.pack()

def main():
    while clickModel() == True:
        return True


if __name__ == '__main__':
    main()