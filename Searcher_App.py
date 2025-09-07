import tkinter as tk
from PIL import Image, ImageTk
import cv2
import math
from ultralytics import YOLO
from PIL import Image as PILImage, ImageTk

# Load YOLOv8 model
model = YOLO("best.pt")
desired_item = "red_square"

# Define object classes
classNames = ["green_circle", "red_square", "yellow_triangle"]

#Define drop down menu options
BinList = ["1", "2", "3"]

#Define starting coordinates
AbsoluteX = 0.00
AbsoluteY = 0.00
AbsoluteZ = 0.00
AbsoluteU = 0.00

IncrementalX = 0.00
IncrementalY = 0.00
IncrementalZ = 0.00
IncrementalU = 0.00

#Define UI Booleans
Connected_To_Searcher = False
Servo_Power = False
Manual_Control = False
Suction = False
Item_Is_Selected = False
From_Bin_Is_Selected = False
To_Bin_Is_Selected = False
Flashlight_Is_On = False
In_AutoMovement = False

def show_window():
    root.deiconify()
#    print("Window is now visible")

#Test command
def test_command():
    print("Yay!")

class WebcamApp:
    def __init__(self, parent):
        self.frame_skip = 4  # Process every 3rd frame
        self.frame_count = 0

        self.window = parent

        # Initialize a single video stream
        self.video_capture = cv2.VideoCapture(0)
        self.video_capture.set(3, 640)
        self.video_capture.set(4, 640)

        self.canvas_frame = tk.Frame(self.window, bg="#c0e6e2", padx=10, pady=10)
        self.canvas_frame.pack()

        self.canvas = tk.Canvas(self.canvas_frame, width=640, height=480, highlightthickness=0, bd=0, bg="#c0e6e2")
        self.canvas.pack()

        # Placeholder image
        self.image_on_canvas = self.canvas.create_image(0, 0, anchor=tk.NW)

        # Clean up on exit
        self.window.bind("<Destroy>", lambda e: self.on_closing())

        self.update_webcam()

    def update_webcam(self):
        ret, frame = self.video_capture.read()
        self.frame_count += 1

        if ret and self.frame_count % self.frame_skip == 0:
            item_count = 0
            frame = cv2.resize(frame, (640, 480))
            results = model(frame, stream=True)

            for r in results:
                for box in r.boxes:
                    x1, y1, x2, y2 = map(int, box.xyxy[0])
                    confidence = math.ceil((box.conf[0] * 100)) / 100
                    cls = int(box.cls[0])
                    if cls < len(classNames):
                        label = classNames[cls]
                    else:
                        label = f"class_{cls}"  # fallback label for unknown classes

                    if label == desired_item:
                        item_count += 1

                    # Draw bounding box and label on frame
                    cv2.rectangle(frame, (x1, y1), (x2, y2), (255, 0, 255), 2)
                    cv2.putText(frame, f"{label} {confidence}", (x1, y1 - 10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)

            cv2.putText(frame, f"Items: {item_count}", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

            # Convert frame to RGB and render in Tkinter
            rgb_image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            image = Image.fromarray(rgb_image)
            photo = ImageTk.PhotoImage(image=image)

            self.canvas.itemconfig(self.image_on_canvas, image=photo)
            self.canvas.image = photo  # Avoid garbage collection

        self.window.after(15, self.update_webcam)

    def on_closing(self):
        self.video_capture.release()
        self.window.destroy()

class ToggleSwitch_Grid:
    def __init__(self, master, row, column, command):
        self.master = master
        self.command = command

        #Load Images
        self.on_image = PILImage.open("App_Photos/on_switch_dark_background.png")
        self.on_image = self.on_image.resize((100, 80))
        self.on_image = ImageTk.PhotoImage(self.on_image)
        self.off_image = PILImage.open("App_Photos/off_switch_dark_background.png")
        self.off_image = self.off_image.resize((100, 80))
        self.off_image = ImageTk.PhotoImage(self.off_image)


        self.is_on = False #Initial State


        #Function
        def self_function():
            if self.is_on == False:
                self.button.config(image=self.on_image)
                self.command()
                self.is_on = True
            else:
                self.button.config(image=self.off_image)
                self.is_on = False

        #Load switch into UI
        self.button = tk.Label(master, image=self.off_image, bg="#011f26")
        self.button.grid(row=row, column=column)

        #Switch Pressing
        self.button.bind("<Button-1>", lambda event: self_function())

class ToggleSwitch_Place:
    def __init__(self, master, x, y, command):
        self.master = master
        self.command = command

        #Load Images
        self.on_image = PILImage.open("App_Photos/on_switch_dark_background.png")
        self.on_image = self.on_image.resize((100, 80))
        self.on_image = ImageTk.PhotoImage(self.on_image)
        self.off_image = PILImage.open("App_Photos/off_switch_dark_background.png")
        self.off_image = self.off_image.resize((100, 80))
        self.off_image = ImageTk.PhotoImage(self.off_image)


        self.is_on = False #Initial State


        #Function
        def self_function():
            if self.is_on == False:
                self.button.config(image=self.on_image)
                self.command()
                self.is_on = True
            else:
                self.button.config(image=self.off_image)
                self.is_on = False

        #Load switch into UI
        self.button = tk.Label(master, image=self.off_image, bg="#011f26")
        self.button.place(x=x, y=y)

        #Switch Pressing
        self.button.bind("<Button-1>", lambda event: self_function())

class ToggleButton_Grid:
    def press(self, event=None):
        self.is_on = True
        self.button.config(image=self.on_image)
        self.command()

    def release(self, event=None):
        self.is_on = False
        self.button.config(image=self.off_image)

    def __init__(self, master, row, column, command):
        self.master = master
        self.command = command

        #Load Images
        self.on_image = Image.open("App_Photos/on_button_dark_background.png")
        self.on_image = self.on_image.resize((120, 100))
        self.on_image = ImageTk.PhotoImage(self.on_image)
        self.off_image = Image.open("App_Photos/off_button_dark_background.png")
        self.off_image = self.off_image.resize((120, 100))
        self.off_image = ImageTk.PhotoImage(self.off_image)

        self.is_on = False #Initial State

        #Load switch into UI
        self.button = tk.Label(master, image=self.off_image, bg="#011f26")
        self.button.grid(row=row, column=column)

        # Bind Press and Release Events
        self.button.bind("<ButtonPress-1>", self.press)
        self.button.bind("<ButtonRelease-1>", self.release)


class ToggleButton_Place:
    def press(self, event=None):
        self.is_on = True
        self.button.config(image=self.on_image)
        self.command()

    def release(self, event=None):
        self.is_on = False
        self.button.config(image=self.off_image)

    def __init__(self, master, x, y, command):
        self.master = master
        self.command=command

        #Load Images
        self.on_image = Image.open("App_Photos/on_button_dark_background.png")
        self.on_image = self.on_image.resize((120, 100))
        self.on_image = ImageTk.PhotoImage(self.on_image)
        self.off_image = Image.open("App_Photos/off_button_dark_background.png")
        self.off_image = self.off_image.resize((120, 100))
        self.off_image = ImageTk.PhotoImage(self.off_image)

        self.is_on = False #Initial State

        #Load switch into UI
        self.button = tk.Label(master, image=self.off_image, bg="#011f26")
        self.button.place(x=x, y=y)

        # Bind Press and Release Events
        self.button.bind("<ButtonPress-1>", self.press)
        self.button.bind("<ButtonRelease-1>", self.release)        #Switch Pressing

class ToggleButton_Arrow_Place:

    def release(self, event=None):
        self.is_on = False
        #print("Toggle state:", self.is_on)
        self.button.config(image=self.off_image)
        self.dimension_label.config(bg="#656768", fg="#c0e6e2")


    def __init__(self, master, straight, direction, dimension, x, y, command):
        self.master = master
        self.command = command

        #Load Images
        if direction == "up" or direction == "down" or direction == "left" or direction == "right":
            self.on_image = Image.open("App_Photos/active_"+straight+"_"+direction+"_arrow_dark_background.png")
            self.on_image = self.on_image.resize((120, 100))
            self.on_image = ImageTk.PhotoImage(self.on_image)
            self.off_image = Image.open("App_Photos/ready_"+straight+"_"+direction+"_arrow_dark_background.png")
            self.off_image = self.off_image.resize((120, 100))
            self.off_image = ImageTk.PhotoImage(self.off_image)
        else:
            self.on_image = Image.open("App_Photos/active_"+straight+"_"+direction+"_arrow_dark_background.png")
            self.on_image = self.on_image.resize((110, 110))
            self.on_image = ImageTk.PhotoImage(self.on_image)
            self.off_image = Image.open("App_Photos/ready_"+straight+"_"+direction+"_arrow_dark_background.png")
            self.off_image = self.off_image.resize((110, 110))
            self.off_image = ImageTk.PhotoImage(self.off_image)

        self.is_on = False #Initial State

        #Load switch into UI
        self.button = tk.Label(master, image=self.off_image, bg="#011f26")
        self.button.place(x=x, y=y)
        self.dimension_label = tk.Label(manual_frame, text=dimension,bg = "#656768", fg = "#c0e6e2", font = ("Calibri", 13))
        if direction == "up":
            self.dimension_label.place(x=x+43, y=y+20)
        elif direction == "down":
            self.dimension_label.place(x=x+48, y=y+50)
        elif direction == "left":
            self.dimension_label.place(x=x+30, y=y+37)
        elif direction == "right":
            self.dimension_label.place(x=x+55, y=y+37)
        elif direction == "clockwise":
            self.dimension_label.place(x=x+35, y=y+80)
        else:
            self.dimension_label.place(x=x+47, y=y+80)

        def self_function():
            self.is_on = True
            #print("Toggle state:", self.is_on)
            self.button.config(image=self.on_image)
            self.dimension_label.config(bg="#c0e6e2", fg="#011f26")
            self.command()
        
        # Bind Press and Release Events
        self.button.bind("<ButtonRelease-1>", self.release)        #Switch Pressing
        self.button.bind("<Button-1>", lambda event: self_function())
        self.dimension_label.bind("<ButtonRelease-1>", self.release)        #Switch Pressing
        self.dimension_label.bind("<Button-1>", lambda event: self_function())


if __name__ == "__main__":
    root = tk.Tk()
    root.withdraw()
    root.title("Searcher")
    root.geometry("2400x1600")
    root.configure(bg="#c0e6e2")

    #Main Switches
    mainswitches_frame = tk.Frame(root, width=900, height = 160, bg="#011f26")
    mainswitches_frame.place(x=0, y=0)
    mainswitches_frame.pack_propagate(False)
    mainswitches_frame.grid_propagate(False)

    mainswitches_frame.grid_rowconfigure(0, minsize=20)
    mainswitches_frame.grid_rowconfigure(1, minsize=20)

    mainswitches_frame.grid_columnconfigure(0, minsize=180)
    mainswitches_frame.grid_columnconfigure(1, minsize=180)
    mainswitches_frame.grid_columnconfigure(2, minsize=180)
    mainswitches_frame.grid_columnconfigure(3, minsize=180)
    mainswitches_frame.grid_columnconfigure(4, minsize=180)

    ConnectToSearcher_switch = ToggleSwitch_Grid(mainswitches_frame, 0 , 0, test_command)
    ConnectToSearcher_label = tk.Label(mainswitches_frame, text="Connect to\n Searcher",bg = "#011f26", fg = "#c0e6e2", font = ("Calibri", 13))
    ConnectToSearcher_label.grid(row=1, column=0)

    ServoPower_switch = ToggleSwitch_Grid(mainswitches_frame, 0 , 1, test_command)
    ServoPower_label = tk.Label(mainswitches_frame, text="Servo Power",bg = "#011f26", fg = "#c0e6e2", font = ("Calibri", 13))
    ServoPower_label.grid(row=1, column=1)

    Home_button = ToggleButton_Grid(mainswitches_frame, 0 , 2, test_command)
    Home_label = tk.Label(mainswitches_frame, text="Home",bg = "#011f26", fg = "#c0e6e2", font = ("Calibri", 13))
    Home_label.grid(row=1, column=2)

    FlashLight_switch = ToggleSwitch_Grid(mainswitches_frame, 0 , 3, test_command)
    FlashLight_label = tk.Label(mainswitches_frame, text="Flashlight",bg = "#011f26", fg = "#c0e6e2", font = ("Calibri", 13))
    FlashLight_label.grid(row=1, column=3)

    StoragePosition_button = ToggleButton_Grid(mainswitches_frame, 0 , 4, test_command)
    StoragePosition_label = tk.Label(mainswitches_frame, text="Move to\n Storage Position",bg = "#011f26", fg = "#c0e6e2", font = ("Calibri", 13))
    StoragePosition_label.grid(row=1, column=4)

   #Webcam Frame
    webcam_frame = tk.Frame(root, width=900, height=900, bg = "#c0e6e3")
    webcam_frame.pack_propagate(False)
    webcam_frame.place(x=0, y=160)  # You can use .grid() or .place() as well
    app = WebcamApp(webcam_frame)

    #Terminal Frame
    terminal_frame = tk.Frame(root, width=900, height = 345, bg="#c0e6e2")
    terminal_frame.place(x=0, y=660)
    terminal_frame.pack_propagate(False)
    terminal_frame.grid_propagate(False)

    #Automated Controls Frame
    automated_frame = tk.Frame(root, width=700, height = 380, bg="#011f26", highlightbackground="#c0e6e2", highlightthickness=10)
    automated_frame.place(x=900, y=0)
    automated_frame.pack_propagate(False)
    automated_frame.grid_propagate(False)

    automated_frame.grid_rowconfigure(0, minsize=70)

    automated_frame.grid_columnconfigure(0, minsize=15)
    automated_frame.grid_columnconfigure(1, minsize=70)
    automated_frame.grid_columnconfigure(2, minsize=100)
    automated_frame.grid_columnconfigure(3, minsize=100)
    automated_frame.grid_columnconfigure(4, minsize=100)
    automated_frame.grid_columnconfigure(5, minsize=70)
    automated_frame.grid_columnconfigure(6, minsize=100)
    automated_frame.grid_columnconfigure(7, minsize=20)    


    Move_label = tk.Label(automated_frame, text="Move", bg = "#011f26", fg="#c0e6e2", font = ("Calibri, 13"))
    Move_label.grid(row=0, column=1)
    FromBin_label = tk.Label(automated_frame, text="from bin", bg = "#011f26", fg="#c0e6e2", font = ("Calibri, 13"))
    FromBin_label.grid(row=0, column=3)
    ToBin_label = tk.Label(automated_frame, text="to bin", bg = "#011f26", fg="#c0e6e2", font = ("Calibri, 13"))
    ToBin_label.grid(row=0, column=5)
    Period_label = tk.Label(automated_frame, text=".", bg = "#011f26", fg="#c0e6e2", font = ("Calibri, 13"))
    Period_label.grid(row=0, column=7)

    ObjectToMove = tk.StringVar(automated_frame)
    ObjectToMove.set("") # default value
    ObjectToMoveMenu = tk.OptionMenu(automated_frame, ObjectToMove, "green circle", "red square", "yellow triangle",)
    ObjectToMoveMenu.config(width=10)
    ObjectToMoveMenu.grid(row = 0, column = 2)

    FromBin = tk.StringVar(automated_frame)
    FromBin.set("") # default value
    MoveFromMenu = tk.OptionMenu(automated_frame, FromBin, "1", "2", "3",)
    MoveFromMenu.config(width=10)
    MoveFromMenu.grid(row = 0, column = 4)

    ToBin = tk.StringVar(automated_frame)
    ToBin.set("") # default value
    MoveToMenu = tk.OptionMenu(automated_frame, ToBin, "1", "2", "3",)
    MoveToMenu.config(width=10)
    MoveToMenu.grid(row = 0, column = 6)

    Start_button = ToggleButton_Place(automated_frame, 0 , 75, test_command)
    Start_label = tk.Label(automated_frame, text="Start",bg = "#011f26", fg = "#c0e6e2", font = ("Calibri", 13))
    Start_label.place(x=37, y=170)    

    Stop_button = ToggleButton_Place(automated_frame, 100 , 75, test_command)
    Stop_label = tk.Label(automated_frame, text="Stop",bg = "#011f26", fg = "#c0e6e2", font = ("Calibri", 13))
    Stop_label.place(x=140, y=170)

    AbsoluteCoordinatesBackground_label_image = Image.open("App_Photos/coordinate_box_dark_background.png")
    AbsoluteCoordinatesBackground_label_image = AbsoluteCoordinatesBackground_label_image.resize((150, 150))
    AbsoluteCoordinatesBackground_label_image = ImageTk.PhotoImage(AbsoluteCoordinatesBackground_label_image)
    AbsoluteCoordinatesBackground_label = tk.Label(automated_frame, borderwidth=0, image=AbsoluteCoordinatesBackground_label_image)
    AbsoluteCoordinatesBackground_label.place(x=430, y=170)
    AbsoluteCoordinates_label = tk.Label(automated_frame, highlightthickness=0, highlightcolor="#011f26", fg = "#011f26", bg = "#c0e6e2", text="Absolute\nX: "+str(AbsoluteX)+" in\nY: " +str(AbsoluteY)+" in\nZ: "+str(AbsoluteZ)+" in\nU: "+str(AbsoluteU)+"°", font = ("Calibri", 13))    
    AbsoluteCoordinates_label.place(x=465, y=190)

    #Manual Controls Frame
    manual_frame = tk.Frame(root, width=700, height = 622, bg="#011f26", highlightbackground="#c0e6e2", highlightthickness=10)
    manual_frame.place(x=900, y=380)  
    manual_frame.pack_propagate(False)  
    manual_frame.grid_propagate(False)

    manual_frame.grid_columnconfigure(0, minsize=15)

    ManualControl_switch = ToggleSwitch_Grid(manual_frame, 0 , 1, test_command)
    ManualControl_label = tk.Label(manual_frame, text="Manual\nControl",bg = "#011f26", fg = "#c0e6e2", font = ("Calibri", 13))
    ManualControl_label.grid(row=1, column=1)

    Suction_switch = ToggleSwitch_Grid(manual_frame, 2 , 1, test_command)
    Suction_label = tk.Label(manual_frame, text="Suction",bg = "#011f26", fg = "#c0e6e2", font = ("Calibri", 13))
    Suction_label.grid(row=3, column=1)

    IncrementalCoordinatesBackground_label_image = Image.open("App_Photos/coordinate_box_dark_background.png")
    IncrementalCoordinatesBackground_label_image = IncrementalCoordinatesBackground_label_image.resize((150, 150))
    IncrementalCoordinatesBackground_label_image = ImageTk.PhotoImage(IncrementalCoordinatesBackground_label_image)
    IncrementalCoordinatesBackground_label = tk.Label(manual_frame, borderwidth=0, image=IncrementalCoordinatesBackground_label_image)
    IncrementalCoordinatesBackground_label.place(x=10, y=280)
    IncrementalCoordinates_label = tk.Label(manual_frame, highlightthickness=0, highlightcolor="#011f26", fg = "#011f26", bg = "#c0e6e2", text="Incremental\nX: "+str(IncrementalX)+" in\nY: " +str(IncrementalY)+" in\nZ: "+str(IncrementalZ)+" in\nU: "+str(IncrementalU)+"°", font = ("Calibri", 13))    
    IncrementalCoordinates_label.place(x=25, y=300)

    Positive_Z_switch = ToggleButton_Arrow_Place(manual_frame,"straight","up","+Z", 525 , 120, test_command)
    Negative_Z_switch = ToggleButton_Arrow_Place(manual_frame,"straight","down","-Z", 525 , 240, test_command)

    Positive_Y_switch = ToggleButton_Arrow_Place(manual_frame,"straight","up","+Y", 320 , 200, test_command)
    Negative_Y_switch = ToggleButton_Arrow_Place(manual_frame,"straight","down","-Y", 320 , 480, test_command)

    Positive_X_switch = ToggleButton_Arrow_Place(manual_frame,"straight","right","+X", 475 , 340, test_command)
    Negative_X_switch = ToggleButton_Arrow_Place(manual_frame,"straight","left","-X", 165, 340, test_command)

    Positive_U_switch = ToggleButton_Arrow_Place(manual_frame,"curved","counterclockwise","+U", 270 , 340, test_command)
    Negative_U_switch = ToggleButton_Arrow_Place(manual_frame,"curved","clockwise","-U", 380 , 340, test_command)

    Zero_X_button = ToggleButton_Place(manual_frame, 120, 0, test_command)
    Zero_X_label = tk.Label(manual_frame, text="Zero X",bg = "#011f26", fg = "#c0e6e2", font = ("Calibri", 13))
    Zero_X_label.place(x=150, y=90)

    Zero_Y_button = ToggleButton_Place(manual_frame, 210 , 0, test_command)
    Zero_Y_label = tk.Label(manual_frame, text="Zero Y",bg = "#011f26", fg = "#c0e6e2", font = ("Calibri", 13))
    Zero_Y_label.place(x=240, y=90)

    Zero_Z_button = ToggleButton_Place(manual_frame, 300 , 0, test_command)
    Zero_Z_label = tk.Label(manual_frame, text="Zero Z",bg = "#011f26", fg = "#c0e6e2", font = ("Calibri", 13))
    Zero_Z_label.place(x=330, y=90)

    Zero_U_button = ToggleButton_Place(manual_frame, 390 , 0, test_command)
    Zero_U_label = tk.Label(manual_frame, text="Zero U",bg = "#011f26", fg = "#c0e6e2", font = ("Calibri", 13))
    Zero_U_label.place(x=420, y=90)

    root.after(100, show_window)
    root.mainloop()