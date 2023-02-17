"""
#           MUIIR (Máster Universitario Informática Industrial y Robótica)                          #
#                                                                                                   #
#                                                                                                   #
#                            @author: Daniel Cardezo                                                #
#                                                                                                   #
#                                                                                                   #
#                            @co-tutor: Alma Mallo                                                  #
#                            @co-tutor: Alejandro Paz                                               #
"""

"""           Graphic User Interface for Computer Vision System Development                        """

# Library importation
import customtkinter
import os
from PIL import Image
import tkinter
from tkinter import filedialog


class App(customtkinter.CTk):
    def __init__(self):
        super().__init__()

        # insert the title of the project
        self.title("Asistente para crear una solución de reconocimiento de objetos")
        self.geometry("700x450")

        # set grid layout 1x2
        self.grid_rowconfigure(0, weight=1)
        self.grid_columnconfigure(1, weight=1)

        # images directory
        image_path = os.path.join(os.path.dirname(os.path.realpath(__file__)), "images")

        """                                            IMAGE LOADING                                 
                COMPONENTS: ALL IMAGE DEFINITIONS IN ORDER TO BE USED ALONG THE GUI
                FUNCTION OF THE FRAME: Load images with light and dark mode image
        """

        self.flecha_derecha = customtkinter.CTkImage(
            Image.open(os.path.join(image_path, "flecha-correcta.png")), size=(10, 10)
        )

        self.flecha_izquierda = customtkinter.CTkImage(
            Image.open(os.path.join(image_path, "flecha-izquierda.png")), size=(10, 10)
        )

        self.large_test_image = customtkinter.CTkImage(
            Image.open(os.path.join(image_path, "IMAGEN_LARGA.png")), size=(500, 200)
        )

        self.image_icon_image = customtkinter.CTkImage(
            Image.open(os.path.join(image_path, "image_icon_light.png")), size=(20, 20)
        )

        self.home_image = customtkinter.CTkImage(
            light_image=Image.open(os.path.join(image_path, "home_dark.png")),
            dark_image=Image.open(os.path.join(image_path, "home_light.png")),
            size=(20, 20),
        )

        self.chat_image = customtkinter.CTkImage(
            light_image=Image.open(os.path.join(image_path, "chat_dark.png")),
            dark_image=Image.open(os.path.join(image_path, "chat_light.png")),
            size=(20, 20),
        )

        self.add_user_config = customtkinter.CTkImage(
            light_image=Image.open(os.path.join(image_path, "config_black.png")),
            dark_image=Image.open(os.path.join(image_path, "config_white.png")),
            size=(20, 20),
        )

        self.add_user_image = customtkinter.CTkImage(
            light_image=Image.open(os.path.join(image_path, "add_user_dark.png")),
            dark_image=Image.open(os.path.join(image_path, "add_user_light.png")),
            size=(20, 20),
        )

        self.successful_image = customtkinter.CTkImage(
            light_image=Image.open(os.path.join(image_path, "configuracion.png")),
            dark_image=Image.open(os.path.join(image_path, "success_white.png")),
            size=(400, 400),
        )

        self.solution_image = customtkinter.CTkImage(
            light_image=Image.open(os.path.join(image_path, "ocurrencia_black.png")),
            dark_image=Image.open(os.path.join(image_path, "ocurrencia_white.png")),
            size=(20, 20),
        )

        self.init_image = customtkinter.CTkImage(
            light_image=Image.open(os.path.join(image_path, "inicio_black.png")),
            dark_image=Image.open(os.path.join(image_path, "inicio_white.png")),
            size=(300, 300),
        )

        """                                            MENU BUTTONS                                 
                COMPONENTS:
                FUNCTION OF THE FRAME: CREATION AND CONFIGURATION OF THE LEFT MENU FRAME
        """

        # create navigation frame
        self.navigation_frame = customtkinter.CTkFrame(self, corner_radius=0)
        self.navigation_frame.grid(row=0, column=0, sticky="nsew")
        self.navigation_frame.grid_rowconfigure(6, weight=1)

        # title of the navigarion frame
        self.navigation_frame_label = customtkinter.CTkLabel(
            self.navigation_frame,
            text="Menú",
            compound="bottom",
            font=customtkinter.CTkFont(size=15, weight="bold"),
        )
        self.navigation_frame_label.grid(row=0, column=0, padx=20, pady=20)

        # init button
        self.home_button = customtkinter.CTkButton(
            self.navigation_frame,
            corner_radius=0,
            height=40,
            border_spacing=10,
            text="Inicio",
            fg_color="transparent",
            text_color=("gray10", "gray90"),
            hover_color=("gray70", "gray30"),
            image=self.home_image,
            anchor="w",
            command=self.home_button_event,
        )
        self.home_button.grid(row=1, column=0, sticky="ew")

        # techniques button
        self.techniques_button = customtkinter.CTkButton(
            self.navigation_frame,
            corner_radius=0,
            height=40,
            border_spacing=10,
            text="Técnicas",
            fg_color="transparent",
            text_color=("gray10", "gray90"),
            hover_color=("gray70", "gray30"),
            image=self.chat_image,
            anchor="w",
            command=self.techniques_button_event,
        )
        self.techniques_button.grid(row=2, column=0, sticky="ew")

        # models button
        self.models_button = customtkinter.CTkButton(
            self.navigation_frame,
            corner_radius=0,
            height=40,
            border_spacing=10,
            text="Modelos",
            fg_color="transparent",
            text_color=("gray10", "gray90"),
            hover_color=("gray70", "gray30"),
            image=self.add_user_image,
            anchor="w",
            command=self.models_button_event,
        )
        self.models_button.grid(row=3, column=0, sticky="ew")

        # config button
        self.config_button = customtkinter.CTkButton(
            self.navigation_frame,
            corner_radius=0,
            height=40,
            border_spacing=10,
            text="Configuración",
            fg_color="transparent",
            text_color=("gray10", "gray90"),
            hover_color=("gray70", "gray30"),
            image=self.add_user_config,
            anchor="w",
            command=self.config_button_event,
        )
        self.config_button.grid(row=4, column=0, sticky="ew")

        # solution button
        self.solution_button = customtkinter.CTkButton(
            self.navigation_frame,
            corner_radius=0,
            height=40,
            border_spacing=10,
            text="Solución",
            fg_color="transparent",
            text_color=("gray10", "gray90"),
            hover_color=("gray70", "gray30"),
            image=self.solution_image,
            anchor="w",
            command=self.solution_button_event,
        )
        self.solution_button.grid(row=5, column=0, sticky="ew")

        # in order to select different GUI modes
        self.appearance_mode_menu = customtkinter.CTkOptionMenu(
            self.navigation_frame, values=["Light", "Dark", "System"], command=self.change_appearance_mode_event
        )
        self.appearance_mode_menu.grid(row=6, column=0, padx=20, pady=20, sticky="s")

        """                                            HOME FRAME                                 
                COMPONENTS:
                FUNCTION OF THE FRAME: CREATION AND CONFIGURATION OF THE HOME INIT FRAME
        """

        # create home frame
        self.home_frame = customtkinter.CTkFrame(self, corner_radius=0, fg_color="transparent")
        self.home_frame.grid_columnconfigure(2, weight=1)
        self.home_frame.grid_rowconfigure(3, weight=1)

        # title of the home frame
        self.home_frame_title = customtkinter.CTkLabel(
            self.home_frame,
            text="ASISTENTE PARA CREAR SOLUCIÓN DE RECONOCIMIENTO DE OBJETOS",
            compound="bottom",
            image=self.init_image,
            fg_color="transparent",
            font=customtkinter.CTkFont(size=20, weight="bold"),
        )
        self.home_frame_title.grid(row=0, column=0, padx=120, pady=10)

        # create textbox
        self.textbox = customtkinter.CTkTextbox(self.home_frame, width=350, height=250)
        self.textbox.grid(row=1, column=0, padx=(20, 0), pady=(10, 0), sticky="nsew")
        self.textbox.insert(
            "0.0",
            "Descripción\n\n"
            + "Esta interfaz de usuario permite entrenar sistemas para detección de objetos.\n\n"
            + "Es necesario por parte del usuario introducir un conjunto de datos etiquetados. \n\n"
            + "Existen distintas opciones para diseñar el sistema de detección de objetos, el usuario deberá seleccionar si desea: \n\n"
            + "- Reconocimiento de objetos por plataformas en la nube. \n\n"
            + "- Reconocimiento de objetos por redes neuronales locales. \n\n"
            + "- Reconocimiento de objetos por el uso de métodos clásicos. \n\n"
            + "Una vez terminada la configuración del modelo, se exportará un fichero en la carpeta de destino definida por el usuario. \n\n"
            + "Para crear un modelo de detección de objetos, presione el botón 'Empezar'. \n\n",
        )

        # button in order to start the assistant
        self.home_start_button = customtkinter.CTkButton(
            self.home_frame,
            text=" Empezar ",
            width=200,
            height=350,
            fg_color="DarkCyan",
            command=self.start_button_event,
        )
        self.home_start_button.grid(row=1, column=1, padx=20, pady=10)

        """                                            TECHNIQUE SELECTION FRAME                                 
                COMPONENTS:
                FUNCTION OF THE FRAME: CREATION AND CONFIGURATION OF THE TECHNIQUES SELECTION FRAME
        """

        # create tech frame
        self.tech_frame = customtkinter.CTkFrame(self, corner_radius=0, fg_color="transparent")
        self.tech_frame.grid_columnconfigure(5, weight=1)
        self.tech_frame.grid_rowconfigure(3, weight=1)

        self.tech_frame_title = customtkinter.CTkButton(
            self.tech_frame, text=" TÉCNICAS ", fg_color="DarkCyan", width=400, height=150
        )
        self.tech_frame_title.grid(row=0, column=2, padx=20, pady=30)

        self.tech_button_neural_net = customtkinter.CTkButton(
            self.tech_frame,
            text=" Redes neuronales ",
            width=300,
            height=200,
            fg_color="DarkCyan",
            command=self.neural_button_event,
        )
        self.tech_button_neural_net.grid(row=1, column=1, padx=20, pady=80)

        self.tech_button_google = customtkinter.CTkButton(
            self.tech_frame,
            text=" Servicios en la nube ",
            width=300,
            height=200,
            fg_color="DarkCyan",
            command=self.google_button_event,
        )
        self.tech_button_google.grid(row=1, column=2, padx=20, pady=80)

        self.tech_button_classique = customtkinter.CTkButton(
            self.tech_frame,
            text=" Técnicas clásicas ",
            width=300,
            height=200,
            fg_color="DarkCyan",
            command=self.classique_button_event,
        )
        self.tech_button_classique.grid(row=1, column=3, padx=20, pady=80)

        self.tech_button_before = customtkinter.CTkButton(
            self.tech_frame,
            text=" Anterior ",
            width=30,
            height=50,
            command=self.before_button_event_1,
            image=self.flecha_izquierda,
            fg_color="DarkCyan",
            compound="left",
        )

        self.tech_button_before.grid(row=2, column=1, padx=20, pady=10)

        self.tech_button_after = customtkinter.CTkButton(
            self.tech_frame,
            text=" Siguiente ",
            width=30,
            height=50,
            command=self.next_button_event_1,
            image=self.flecha_derecha,
            fg_color="DarkCyan",
            compound="right",
        )
        self.tech_button_after.grid(row=2, column=3, padx=60, pady=10)

        """                                            MODELS FRAME (YOLO)                             
                COMPONENTS:
                FUNCTION OF THE FRAME: CREATION AND CONFIGURATION OF THE NEURAL NETWORK MODEL SELECTION FRAME
        """

        # create models frame
        self.models_yolo_frame = customtkinter.CTkFrame(self, corner_radius=0, fg_color="transparent")
        self.models_yolo_frame.grid_columnconfigure(5, weight=1)
        self.models_yolo_frame.grid_rowconfigure(3, weight=1)

        # define the title of the models frame
        self.models_yolo_frame_title = customtkinter.CTkButton(
            self.models_yolo_frame, text=" MODELOS ", fg_color="DarkCyan", width=400, height=150
        )
        self.models_yolo_frame_title.grid(row=0, column=2, padx=20, pady=30)

        # button in order to select the YOLOv5 neural net
        self.tech_button_yolo = customtkinter.CTkButton(
            self.models_yolo_frame,
            text=" YOLOv5 ",
            width=300,
            height=200,
            fg_color="DarkCyan",
            command=self.yolo_button_event,
        )
        self.tech_button_yolo.grid(row=1, column=1, padx=5, pady=80)

        # button in order to select HOG classique technique
        self.tech_button_model2_yolo = customtkinter.CTkButton(
            self.models_yolo_frame,
            text=" Modelo 2 ",
            width=300,
            height=200,
            fg_color="DarkCyan",
            command=self.m2_yolo_button_event,
        )
        self.tech_button_model2_yolo.grid(row=1, column=2, padx=5, pady=80)

        # button in order to select the Google Cloud Vision API
        self.tech_button_model3_yolo = customtkinter.CTkButton(
            self.models_yolo_frame,
            text=" Modelo 3 ",
            width=300,
            height=200,
            fg_color="DarkCyan",
            command=self.m3_yolo_button_event,
        )
        self.tech_button_model3_yolo.grid(row=1, column=3, padx=5, pady=80)

        # button in order to return to select the technique
        self.models_button_before_yolo = customtkinter.CTkButton(
            self.models_yolo_frame,
            text=" Anterior ",
            width=30,
            height=50,
            command=self.before_button_event_yolo,
            image=self.flecha_izquierda,
            fg_color="DarkCyan",
            compound="left",
        )
        self.models_button_before_yolo.grid(row=2, column=1, padx=5, pady=10)

        # button in order to pass trow the configuration frame
        self.models_button_after_yolo = customtkinter.CTkButton(
            self.models_yolo_frame,
            text=" Siguiente ",
            width=30,
            height=50,
            command=self.next_button_event_yolo,
            image=self.flecha_derecha,
            fg_color="DarkCyan",
            compound="right",
        )
        self.models_button_after_yolo.grid(row=2, column=3, padx=5, pady=10)

        # button in order to move between different models in a future
        self.models_button_left_yolo = customtkinter.CTkButton(
            self.models_yolo_frame,
            text=" ",
            width=30,
            height=30,
            command=self.left_button_models_yolo,
            fg_color="DarkCyan",
            image=self.flecha_izquierda,
        )
        self.models_button_left_yolo.grid(row=1, column=0, padx=5, pady=10)

        # button in order to move between different models in a future
        self.models_button_right_yolo = customtkinter.CTkButton(
            self.models_yolo_frame,
            text=" ",
            width=30,
            height=30,
            command=self.right_button_models_yolo,
            fg_color="DarkCyan",
            image=self.flecha_derecha,
        )
        self.models_button_right_yolo.grid(row=1, column=4, padx=5, pady=10)

        """                                            MODELS FRAME (HOG)                                
                COMPONENTS:
                FUNCTION OF THE FRAME: CREATION AND CONFIGURATION OF THE CLASSIQUE TECHNIQUES MODEL SELECTION FRAME
        """

        # create models frame
        self.models_hog_frame = customtkinter.CTkFrame(self, corner_radius=0, fg_color="transparent")
        self.models_hog_frame.grid_columnconfigure(5, weight=1)
        self.models_hog_frame.grid_rowconfigure(3, weight=1)

        # define the title of the models frame
        self.models_hog_frame_title = customtkinter.CTkButton(
            self.models_hog_frame, text=" MODELOS ", fg_color="DarkCyan", width=400, height=150
        )
        self.models_hog_frame_title.grid(row=0, column=2, padx=20, pady=30)

        # button in order to select HOG classique technique
        self.tech_button_hog = customtkinter.CTkButton(
            self.models_hog_frame,
            text=" HOG ",
            width=300,
            height=200,
            fg_color="DarkCyan",
            command=self.hog_button_event,
        )
        self.tech_button_hog.grid(row=1, column=1, padx=5, pady=80)

        # button in order to select model 2
        self.tech_button_model2_hog = customtkinter.CTkButton(
            self.models_hog_frame,
            text=" Modelo 2 ",
            width=300,
            height=200,
            fg_color="DarkCyan",
            command=self.m2_hog_button_event,
        )
        self.tech_button_model2_hog.grid(row=1, column=2, padx=5, pady=80)

        # button in order to select model 3
        self.tech_button_model3_hog = customtkinter.CTkButton(
            self.models_hog_frame,
            text=" Modelo 3 ",
            width=300,
            height=200,
            fg_color="DarkCyan",
            command=self.m3_hog_button_event,
        )
        self.tech_button_model3_hog.grid(row=1, column=3, padx=5, pady=80)

        # button in order to return to select the technique
        self.models_button_before_hog = customtkinter.CTkButton(
            self.models_hog_frame,
            text=" Anterior ",
            width=30,
            height=50,
            command=self.before_button_event_hog,
            image=self.flecha_izquierda,
            fg_color="DarkCyan",
            compound="left",
        )
        self.models_button_before_hog.grid(row=2, column=1, padx=5, pady=10)

        # button in order to pass trow the configuration frame
        self.models_button_after_hog = customtkinter.CTkButton(
            self.models_hog_frame,
            text=" Siguiente ",
            width=30,
            height=50,
            command=self.next_button_event_hog,
            image=self.flecha_derecha,
            fg_color="DarkCyan",
            compound="right",
        )
        self.models_button_after_hog.grid(row=2, column=3, padx=5, pady=10)

        # button in order to move between different models in a future
        self.models_button_left_hog = customtkinter.CTkButton(
            self.models_hog_frame,
            text=" ",
            width=30,
            height=30,
            command=self.left_button_models_hog,
            fg_color="DarkCyan",
            image=self.flecha_izquierda,
        )
        self.models_button_left_hog.grid(row=1, column=0, padx=5, pady=10)

        # button in order to move between different models in a future
        self.models_button_right_hog = customtkinter.CTkButton(
            self.models_hog_frame,
            text=" ",
            width=30,
            height=30,
            command=self.right_button_models_hog,
            fg_color="DarkCyan",
            image=self.flecha_derecha,
        )
        self.models_button_right_hog.grid(row=1, column=4, padx=5, pady=10)

        """                                            MODELS FRAME (CLOUD)                                
                COMPONENTS:
                FUNCTION OF THE FRAME: CREATION AND CONFIGURATION OF THE CLOUD MODEL SELECTION FRAME
        """

        # create models frame
        self.models_cloud_frame = customtkinter.CTkFrame(self, corner_radius=0, fg_color="transparent")
        self.models_cloud_frame.grid_columnconfigure(5, weight=1)
        self.models_cloud_frame.grid_rowconfigure(3, weight=1)

        # define the title of the models frame
        self.models_cloud_frame_title = customtkinter.CTkButton(
            self.models_cloud_frame, text=" MODELOS ", fg_color="DarkCyan", width=400, height=150
        )
        self.models_cloud_frame_title.grid(row=0, column=2, padx=20, pady=30)

        # button in order to select Google Cloud Vision API
        self.tech_button_cloud = customtkinter.CTkButton(
            self.models_cloud_frame,
            text=" Google API ",
            width=300,
            height=200,
            fg_color="DarkCyan",
            command=self.cloud_button_event,
        )
        self.tech_button_cloud.grid(row=1, column=1, padx=5, pady=80)

        # button in order to select the model 2
        self.tech_button_model2_cloud = customtkinter.CTkButton(
            self.models_cloud_frame,
            text=" Modelo 2 ",
            width=300,
            height=200,
            fg_color="DarkCyan",
            command=self.m2_cloud_button_event,
        )
        self.tech_button_model2_cloud.grid(row=1, column=2, padx=5, pady=80)

        # button in order to select the model 3
        self.tech_button_model3_cloud = customtkinter.CTkButton(
            self.models_cloud_frame,
            text=" Modelo 3 ",
            width=300,
            height=200,
            fg_color="DarkCyan",
            command=self.m3_cloud_button_event,
        )
        self.tech_button_model3_cloud.grid(row=1, column=3, padx=5, pady=80)

        # button in order to return to select the technique
        self.models_button_before_cloud = customtkinter.CTkButton(
            self.models_cloud_frame,
            text=" Anterior ",
            width=30,
            height=50,
            command=self.before_button_event_cloud,
            image=self.flecha_izquierda,
            fg_color="DarkCyan",
            compound="left",
        )
        self.models_button_before_cloud.grid(row=2, column=1, padx=5, pady=10)

        # button in order to pass trow the configuration frame
        self.models_button_after_cloud = customtkinter.CTkButton(
            self.models_cloud_frame,
            text=" Siguiente ",
            width=30,
            height=50,
            command=self.next_button_event_cloud,
            image=self.flecha_derecha,
            fg_color="DarkCyan",
            compound="right",
        )
        self.models_button_after_cloud.grid(row=2, column=3, padx=5, pady=10)

        # button in order to move between different models in a future
        self.models_button_left_cloud = customtkinter.CTkButton(
            self.models_cloud_frame,
            text=" ",
            width=30,
            height=30,
            command=self.left_button_models_cloud,
            fg_color="DarkCyan",
            image=self.flecha_izquierda,
        )
        self.models_button_left_cloud.grid(row=1, column=0, padx=5, pady=10)

        # button in order to move between different models in a future
        self.models_button_right_cloud = customtkinter.CTkButton(
            self.models_cloud_frame,
            text=" ",
            width=30,
            height=30,
            command=self.right_button_models_cloud,
            fg_color="DarkCyan",
            image=self.flecha_derecha,
        )
        self.models_button_right_cloud.grid(row=1, column=4, padx=5, pady=10)

        """                                            CONFIG FRAME FOR YOLOV5                                
                COMPONENTS:
                FUNCTION OF THE FRAME: CREATION AND CONFIGURATION OF THE YOLOV5 CONFIGURATION FRAME
        """

        # create configuration frame
        self.config_frame_yolov5 = customtkinter.CTkFrame(self, corner_radius=0, fg_color="transparent")
        self.config_frame_yolov5.grid_columnconfigure(3, weight=1)
        self.config_frame_yolov5.grid_rowconfigure(7, weight=1)

        # create training path entry
        self.label_test_training_yolov5 = customtkinter.CTkLabel(
            self.config_frame_yolov5, text="Imágenes entrenamiento:", font=customtkinter.CTkFont(size=15, weight="bold")
        )
        self.label_test_training_yolov5.grid(row=0, column=0, columnspan=1, padx=(20, 0), pady=(20, 40), sticky="nsew")

        self.path_train_yolov5_text = tkinter.StringVar()
        self.path_train_yolov5 = customtkinter.CTkEntry(
            self.config_frame_yolov5,
            textvariable=self.path_train_yolov5_text,
            placeholder_text="Path imágenes de entrenamiento",
            width=800,
            height=30,
        )
        self.path_train_yolov5.grid(row=0, column=1, columnspan=1, padx=(20, 0), pady=(20, 20), sticky="nsew")

        self.path_train_yolov5_button = customtkinter.CTkButton(
            self.config_frame_yolov5,
            text="Select folder",
            width=30,
            height=10,
            command=self.open_file_dialog_training_yolov5,
            fg_color="DarkCyan",
        )
        self.path_train_yolov5_button.grid(row=0, column=2, columnspan=1, padx=(20, 0), pady=(20, 20), sticky="nsew")

        # create result path entry
        self.label_test_result_yolov5 = customtkinter.CTkLabel(
            self.config_frame_yolov5, text="Imágenes resultado:", font=customtkinter.CTkFont(size=15, weight="bold")
        )
        self.label_test_result_yolov5.grid(row=1, column=0, columnspan=1, padx=(20, 0), pady=(20, 40), sticky="nsew")

        self.path_result_yolov5_text = tkinter.StringVar()
        self.path_result_yolov5 = customtkinter.CTkEntry(
            self.config_frame_yolov5,
            textvariable=self.path_result_yolov5_text,
            placeholder_text="Path imágenes resultado",
            width=800,
            height=30,
        )
        self.path_result_yolov5.grid(row=1, column=1, columnspan=1, padx=(20, 0), pady=(20, 20), sticky="nsew")

        self.path_result_yolov5_button = customtkinter.CTkButton(
            self.config_frame_yolov5,
            text="Select folder",
            width=30,
            height=10,
            command=self.open_file_dialog_result_yolov5,
            fg_color="DarkCyan",
        )
        self.path_result_yolov5_button.grid(row=1, column=2, columnspan=1, padx=(20, 0), pady=(20, 20), sticky="nsew")

        # create inference path entry
        self.label_test_inference_yolov5 = customtkinter.CTkLabel(
            self.config_frame_yolov5,
            text="Imágenes inferencia:",
            font=customtkinter.CTkFont(size=15, weight="bold"),
        )
        self.label_test_inference_yolov5.grid(row=3, column=0, columnspan=1, padx=(20, 0), pady=(20, 40), sticky="nsew")

        self.path_inference_yolov5_text = tkinter.StringVar()
        self.path_inference_yolov5 = customtkinter.CTkEntry(
            self.config_frame_yolov5,
            textvariable=self.path_inference_yolov5_text,
            placeholder_text="Path imágenes para inferencia",
            width=800,
            height=30,
        )
        self.path_inference_yolov5.grid(row=3, column=1, columnspan=1, padx=(20, 0), pady=(20, 20), sticky="nsew")

        self.path_inference_yolov5_button = customtkinter.CTkButton(
            self.config_frame_yolov5,
            text="Select folder",
            width=30,
            height=10,
            command=self.open_file_dialog_inference_yolov5,
            fg_color="DarkCyan",
        )
        self.path_inference_yolov5_button.grid(
            row=3, column=2, columnspan=1, padx=(20, 0), pady=(20, 20), sticky="nsew"
        )

        # create epochs number entry
        self.label_test_epochs_yolov5 = customtkinter.CTkLabel(
            self.config_frame_yolov5, text="Número de Epochs:", font=customtkinter.CTkFont(size=15, weight="bold")
        )
        self.label_test_epochs_yolov5.grid(row=4, column=0, columnspan=1, padx=(20, 0), pady=(20, 40), sticky="nsew")

        self.number_epochs_yolov5 = customtkinter.CTkEntry(
            self.config_frame_yolov5, placeholder_text="Número de EPOCHS", width=600, height=30
        )
        self.number_epochs_yolov5.grid(row=4, column=1, columnspan=2, padx=(20, 0), pady=(20, 40), sticky="nsew")

        # create seeed number entry
        self.label_test_seed_yolov5 = customtkinter.CTkLabel(
            self.config_frame_yolov5, text="Número de SEED:", font=customtkinter.CTkFont(size=15, weight="bold")
        )
        self.label_test_seed_yolov5.grid(row=5, column=0, columnspan=1, padx=(20, 0), pady=(20, 180), sticky="nsew")

        self.number_seed_yolov5 = customtkinter.CTkEntry(
            self.config_frame_yolov5, placeholder_text="Número de SEED", width=600, height=30
        )
        self.number_seed_yolov5.grid(row=5, column=1, columnspan=2, padx=(20, 0), pady=(20, 180), sticky="nsew")

        # button in order to return to select the technique
        self.config_button_before_yolov5 = customtkinter.CTkButton(
            self.config_frame_yolov5,
            text=" Anterior ",
            width=30,
            height=50,
            command=self.before_button_event_conf,
            image=self.flecha_izquierda,
            fg_color="DarkCyan",
            compound="left",
        )
        self.config_button_before_yolov5.grid(row=6, column=0, padx=40, pady=5)

        # button in order to pass trow the configuration frame
        self.config_button_after_yolov5 = customtkinter.CTkButton(
            self.config_frame_yolov5,
            text=" Siguiente ",
            width=30,
            height=50,
            command=self.next_button_event_conf,
            image=self.flecha_derecha,
            fg_color="DarkCyan",
            compound="right",
        )
        self.config_button_after_yolov5.grid(row=6, column=2, padx=20, pady=5)

        """                                            CONFIG FRAME FOR HOG                                
                COMPONENTS:
                FUNCTION OF THE FRAME: CREATION AND CONFIGURATION OF THE HOG CONFIGURATION FRAME
        """

        # create configuration frame
        self.config_frame_hog = customtkinter.CTkFrame(self, corner_radius=0, fg_color="transparent")
        self.config_frame_hog.grid_columnconfigure(3, weight=1)
        self.config_frame_hog.grid_rowconfigure(7, weight=1)

        # create training path entry
        self.label_test_training_hog = customtkinter.CTkLabel(
            self.config_frame_hog, text="Imágenes entrenamiento:", font=customtkinter.CTkFont(size=15, weight="bold")
        )
        self.label_test_training_hog.grid(row=0, column=0, columnspan=1, padx=(20, 0), pady=(20, 40), sticky="nsew")

        self.path_train_hog_text = tkinter.StringVar()
        self.path_train_hog = customtkinter.CTkEntry(
            self.config_frame_hog,
            textvariable=self.path_train_hog_text,
            placeholder_text="Path imágenes de entrenamiento",
            width=800,
            height=30,
        )
        self.path_train_hog.grid(row=0, column=1, columnspan=1, padx=(20, 0), pady=(20, 20), sticky="nsew")

        self.path_train_hog_button = customtkinter.CTkButton(
            self.config_frame_hog,
            text="Select folder",
            width=30,
            height=10,
            command=self.open_file_dialog_training_hog,
            fg_color="DarkCyan",
        )
        self.path_train_hog_button.grid(row=0, column=2, columnspan=1, padx=(20, 0), pady=(20, 20), sticky="nsew")

        # create results path entry
        self.label_test_result_hog = customtkinter.CTkLabel(
            self.config_frame_hog, text="Imágenes resultado:", font=customtkinter.CTkFont(size=15, weight="bold")
        )
        self.label_test_result_hog.grid(row=3, column=0, columnspan=1, padx=(20, 0), pady=(20, 40), sticky="nsew")

        self.path_result_hog_text = tkinter.StringVar()
        self.path_result_hog = customtkinter.CTkEntry(
            self.config_frame_hog,
            textvariable=self.path_result_hog_text,
            placeholder_text="Path imágenes de resultado",
            width=800,
            height=30,
        )
        self.path_result_hog.grid(row=3, column=1, columnspan=1, padx=(20, 0), pady=(20, 20), sticky="nsew")

        self.path_result_hog_button = customtkinter.CTkButton(
            self.config_frame_hog,
            text="Select folder",
            width=30,
            height=10,
            command=self.open_file_dialog_result_hog,
            fg_color="DarkCyan",
        )
        self.path_result_hog_button.grid(row=3, column=2, columnspan=1, padx=(20, 0), pady=(20, 20), sticky="nsew")

        # create classifier selection entry
        self.label_test_classifier_hog = customtkinter.CTkLabel(
            self.config_frame_hog,
            text="Selección del clasificador:",
            font=customtkinter.CTkFont(size=15, weight="bold"),
        )
        self.label_test_classifier_hog.grid(row=4, column=0, columnspan=1, padx=(20, 0), pady=(20, 40), sticky="nsew")

        self.selection_classifier_hog = customtkinter.CTkEntry(
            self.config_frame_hog, placeholder_text="Elige entre clasificador SVC o ANN", width=750, height=30
        )
        self.selection_classifier_hog.grid(row=4, column=1, columnspan=2, padx=(20, 0), pady=(20, 40), sticky="nsew")

        # create testing size entry
        self.label_test_size_hog = customtkinter.CTkLabel(
            self.config_frame_hog,
            text="Tamaño del conjunto test:",
            font=customtkinter.CTkFont(size=15, weight="bold"),
        )
        self.label_test_size_hog.grid(row=5, column=0, columnspan=1, padx=(20, 0), pady=(20, 250), sticky="nsew")

        self.test_size_hog = customtkinter.CTkEntry(
            self.config_frame_hog, placeholder_text="Tamaño de conjunto para test (entre 0 y 1)", width=750, height=30
        )
        self.test_size_hog.grid(row=5, column=1, columnspan=2, padx=(20, 0), pady=(20, 250), sticky="nsew")

        # button in order to return to select the technique
        self.config_button_before_hog = customtkinter.CTkButton(
            self.config_frame_hog,
            text=" Anterior ",
            width=30,
            height=50,
            command=self.before_button_event_conf,
            image=self.flecha_izquierda,
            fg_color="DarkCyan",
            compound="left",
        )
        self.config_button_before_hog.grid(row=6, column=0, padx=10, pady=5)

        # button in order to pass trow the configuration frame
        self.config_button_after_hog = customtkinter.CTkButton(
            self.config_frame_hog,
            text=" Siguiente ",
            width=30,
            height=50,
            command=self.next_button_event_conf,
            image=self.flecha_derecha,
            fg_color="DarkCyan",
            compound="right",
        )
        self.config_button_after_hog.grid(row=6, column=2, padx=5, pady=5)

        """                                            CONFIG FRAME FOR CLOUD                                
                COMPONENTS:
                FUNCTION OF THE FRAME: CREATION AND CONFIGURATION OF THE CLOUD CONFIGUATION FRAME
        """

        # create configuration frame
        self.config_frame_cloud = customtkinter.CTkFrame(self, corner_radius=0, fg_color="transparent")
        self.config_frame_cloud.grid_columnconfigure(3, weight=1)
        self.config_frame_cloud.grid_rowconfigure(7, weight=1)

        # create training path entry
        self.label_test_training_cloud = customtkinter.CTkLabel(
            self.config_frame_cloud, text="Imágenes entrenamiento:", font=customtkinter.CTkFont(size=15, weight="bold")
        )
        self.label_test_training_cloud.grid(row=0, column=0, columnspan=1, padx=(20, 0), pady=(20, 40), sticky="nsew")

        self.path_train_cloud_text = tkinter.StringVar()
        self.path_train_cloud = customtkinter.CTkEntry(
            self.config_frame_cloud,
            textvariable=self.path_train_cloud_text,
            placeholder_text="Path imágenes de entrenamiento",
            width=800,
            height=30,
        )
        self.path_train_cloud.grid(row=0, column=1, columnspan=1, padx=(20, 0), pady=(20, 20), sticky="nsew")

        self.path_train_cloud_button = customtkinter.CTkButton(
            self.config_frame_cloud,
            text="Select folder",
            width=30,
            height=10,
            command=self.open_file_dialog_training_cloud,
            fg_color="DarkCyan",
        )
        self.path_train_cloud_button.grid(row=0, column=2, columnspan=1, padx=(20, 0), pady=(20, 20), sticky="nsew")

        # create results path entry
        self.label_test_result_cloud = customtkinter.CTkLabel(
            self.config_frame_cloud, text="Imágenes resultado:", font=customtkinter.CTkFont(size=15, weight="bold")
        )
        self.label_test_result_cloud.grid(row=1, column=0, columnspan=1, padx=(20, 0), pady=(20, 40), sticky="nsew")

        self.path_result_cloud_text = tkinter.StringVar()
        self.path_result_cloud = customtkinter.CTkEntry(
            self.config_frame_cloud,
            textvariable=self.path_result_cloud_text,
            placeholder_text="Path imágenes de resultado",
            width=800,
            height=30,
        )
        self.path_result_cloud.grid(row=1, column=1, columnspan=1, padx=(20, 0), pady=(20, 20), sticky="nsew")

        self.path_result_cloud_button = customtkinter.CTkButton(
            self.config_frame_cloud,
            text="Select folder",
            width=30,
            height=10,
            command=self.open_file_dialog_result_cloud,
            fg_color="DarkCyan",
        )
        self.path_result_cloud_button.grid(row=1, column=2, columnspan=1, padx=(20, 0), pady=(20, 20), sticky="nsew")

        # create object 1 selection entry
        self.label_test_object_1_cloud = customtkinter.CTkLabel(
            self.config_frame_cloud, text="Objeto 1:", font=customtkinter.CTkFont(size=15, weight="bold")
        )
        self.label_test_object_1_cloud.grid(row=2, column=0, columnspan=1, padx=(20, 0), pady=(20, 40), sticky="nsew")

        self.object_1_cloud = customtkinter.CTkEntry(
            self.config_frame_cloud,
            placeholder_text="Seleccione un tipo de objeto a detectar (inglés)",
            width=750,
            height=30,
        )
        self.object_1_cloud.grid(row=2, column=1, columnspan=2, padx=(20, 0), pady=(20, 40), sticky="nsew")

        # create object 2 selection entry
        self.label_test_object_2_cloud = customtkinter.CTkLabel(
            self.config_frame_cloud, text="Objeto 2:", font=customtkinter.CTkFont(size=15, weight="bold")
        )
        self.label_test_object_2_cloud.grid(row=3, column=0, columnspan=1, padx=(20, 0), pady=(20, 250), sticky="nsew")

        self.object_2_cloud = customtkinter.CTkEntry(
            self.config_frame_cloud,
            placeholder_text="Seleccione un tipo de objeto a detectar (opcional) (inglés)",
            width=750,
            height=30,
        )
        self.object_2_cloud.grid(row=3, column=1, columnspan=2, padx=(20, 0), pady=(20, 250), sticky="nsew")

        # button in order to return to select the technique
        self.config_button_before_cloud = customtkinter.CTkButton(
            self.config_frame_cloud,
            text=" Anterior ",
            width=30,
            height=50,
            command=self.before_button_event_conf,
            image=self.flecha_izquierda,
            fg_color="DarkCyan",
            compound="left",
        )
        self.config_button_before_cloud.grid(row=6, column=0, padx=20, pady=5)

        # button in order to pass trow the configuration frame
        self.config_button_after_cloud = customtkinter.CTkButton(
            self.config_frame_cloud,
            text=" Siguiente ",
            width=30,
            height=50,
            command=self.next_button_event_conf,
            image=self.flecha_derecha,
            fg_color="DarkCyan",
            compound="right",
        )
        self.config_button_after_cloud.grid(row=6, column=3, padx=40, pady=5)

        """                                            RESULT FRAME                                 
                COMPONENTS:
                FUNCTION OF THE FRAME: CREATION AND CONFIGURATION OF THE RESULT FRAME
        """

        # Create the result frame
        self.solution_frame = customtkinter.CTkFrame(self, corner_radius=0, fg_color="transparent")
        self.solution_frame.grid_columnconfigure(3, weight=1)
        self.solution_frame.grid_rowconfigure(3, weight=1)

        # success text to confirm that the model has been created correctly
        self.solution_frame_title = customtkinter.CTkLabel(
            self.solution_frame,
            text="¡ENHORABUENA! Tu modelo de reconocimiento de objetos ha sido creado con éxito",
            compound="top",
            image=self.successful_image,
            fg_color="transparent",
            font=customtkinter.CTkFont(size=20, weight="bold"),
        )
        self.solution_frame_title.grid(row=1, column=1, padx=180, pady=100)

        """                                            SELECT DEFAULT FRAME                                 
                COMPONENTS: select_frame_by_name()
                FUNCTION OF THE FRAME: Here, the main frame is defined.
        """
        self.select_frame_by_name("home")

    """                                            TEST OPEN FILE DIALOG                                 
                COMPONENTS: open_file_dialog_training_yolov5(), open_file_dialog_result_yolov5(), open_file_dialog_inference_yolov5(), open_file_dialog_training_hog(), 
                            open_file_dialog_result_hog(), open_file_dialog_training_cloud(), open_file_dialog_result_cloud()
                FUNCTION OF THE FRAME: The functions below are created in order to open a selection window for the different path selections.
    """

    def open_file_dialog_training_yolov5(self):
        self.path_training_file_test = filedialog.askdirectory(initialdir="/", title="Select folder")
        self.path_train_yolov5_text.set(self.path_training_file_test)
        print("----------> Folder name: " + self.path_training_file_test)

    def open_file_dialog_result_yolov5(self):
        self.path_result_file_test = filedialog.askdirectory(initialdir="/", title="Select folder")
        self.path_result_yolov5_text.set(self.path_result_file_test)
        print("----------> Folder name: " + self.path_result_file_test)

    def open_file_dialog_inference_yolov5(self):
        self.path_inference_file_test = filedialog.askdirectory(initialdir="/", title="Select folder")
        self.path_inference_yolov5_text.set(self.path_inference_file_test)
        print("----------> Folder name: " + self.path_inference_file_test)

    def open_file_dialog_training_hog(self):
        self.path_training_file_test = filedialog.askdirectory(initialdir="/", title="Select folder")
        self.path_train_hog_text.set(self.path_training_file_test)
        print("----------> Folder name: " + self.path_training_file_test)

    def open_file_dialog_result_hog(self):
        self.path_result_file_test = filedialog.askdirectory(initialdir="/", title="Select folder")
        self.path_result_hog_text.set(self.path_result_file_test)
        print("----------> Folder name: " + self.path_result_file_test)

    def open_file_dialog_training_cloud(self):
        self.path_training_file_test = filedialog.askdirectory(initialdir="/", title="Select folder")
        self.path_train_cloud_text.set(self.path_training_file_test)
        print("----------> Folder name: " + self.path_training_file_test)

    def open_file_dialog_result_cloud(self):
        self.path_result_file_test = filedialog.askdirectory(initialdir="/", title="Select folder")
        self.path_result_cloud_text.set(self.path_result_file_test)
        print("----------> Folder name: " + self.path_result_file_test)

    """                                            SELECT_FRAME_BY_NAME                                 
            COMPONENTS: select_frame_by_name()
            PURPOSE OF THE FUNCTION: This function selects the frame to show according to the user interaction with 
                                     the different buttons. It also changes the color of the left menu buttons
                                     according to the frame.
    """

    def select_frame_by_name(self, name):

        # set button color for selected button
        self.home_button.configure(fg_color=("gray75", "gray25") if name == "home" else "transparent")
        self.techniques_button.configure(fg_color=("gray75", "gray25") if name == "frame_2" else "transparent")
        self.models_button.configure(
            fg_color=("gray75", "gray25")
            if (name == "neural" or name == "classique" or name == "cloud")
            else "transparent"
        )
        self.config_button.configure(
            fg_color=("gray75", "gray25") if (name == "yolo" or name == "google" or name == "hog") else "transparent"
        )
        self.solution_button.configure(fg_color=("gray75", "gray25") if name == "frame_5" else "transparent")

        # show selected frame
        if name == "home":
            self.home_frame.grid(row=0, column=1, sticky="nsew")
        else:
            self.home_frame.grid_forget()
        if name == "frame_2":
            self.tech_frame.grid(row=0, column=1, sticky="nsew")
        else:
            self.tech_frame.grid_forget()
        if name == "neural":
            self.models_yolo_frame.grid(row=0, column=1, sticky="nsew")
        else:
            self.models_yolo_frame.grid_forget()
        if name == "cloud":
            self.models_cloud_frame.grid(row=0, column=1, sticky="nsew")
        else:
            self.models_cloud_frame.grid_forget()
        if name == "classique":
            self.models_hog_frame.grid(row=0, column=1, sticky="nsew")
        else:
            self.models_hog_frame.grid_forget()
        if name == "yolo":
            self.config_frame_yolov5.grid(row=0, column=1, sticky="nsew")
        else:
            self.config_frame_yolov5.grid_forget()
        if name == "hog":
            self.config_frame_hog.grid(row=0, column=1, sticky="nsew")
        else:
            self.config_frame_hog.grid_forget()
        if name == "google":
            self.config_frame_cloud.grid(row=0, column=1, sticky="nsew")
        else:
            self.config_frame_cloud.grid_forget()
        if name == "frame_5":
            self.solution_frame.grid(row=0, column=1, sticky="nsew")
        else:
            self.solution_frame.grid_forget()

    """                                            SELECT_TECHNIQUE_BY_BUTTON                                 
            COMPONENTS: select_technique_by_button()
            PURPOSE OF THE FUNCTION: This function changes the color of the selected technique when the user presses it.
    """

    def select_technique_by_button(self, button):
        global val
        self.tech_button_google.configure(fg_color=("gray75", "gray25") if button == "cloud" else "DarkCyan")
        self.tech_button_neural_net.configure(fg_color=("gray75", "gray25") if button == "neural" else "DarkCyan")
        self.tech_button_classique.configure(fg_color=("gray75", "gray25") if button == "classique" else "DarkCyan")
        if button == "classique":
            val = 1
        elif button == "neural":
            val = 2
        elif button == "cloud":
            val = 3
        else:
            val = 0

    """                                            SELECT_MODEL_BY_BUTTON                                 
            COMPONENTS: select_model_by_button()
            PURPOSE OF THE FUNCTION: This function changes the color of the selected model when the user presses it.
    """

    def select_model_by_button(self, button):
        global model
        self.tech_button_yolo.configure(fg_color=("gray75", "gray25") if button == "yolo" else "DarkCyan")
        self.tech_button_model2_yolo.configure(
            fg_color=("gray75", "gray25") if button == "Modelo 2 YOLO" else "DarkCyan"
        )
        self.tech_button_model3_yolo.configure(
            fg_color=("gray75", "gray25") if button == "Modelo 3 YOLO" else "DarkCyan"
        )
        self.tech_button_hog.configure(fg_color=("gray75", "gray25") if button == "hog" else "DarkCyan")
        self.tech_button_model2_hog.configure(fg_color=("gray75", "gray25") if button == "Modelo 2 HOG" else "DarkCyan")
        self.tech_button_model3_hog.configure(fg_color=("gray75", "gray25") if button == "Modelo 3 HOG" else "DarkCyan")
        self.tech_button_cloud.configure(fg_color=("gray75", "gray25") if button == "cloud" else "DarkCyan")
        self.tech_button_model2_cloud.configure(
            fg_color=("gray75", "gray25") if button == "Modelo 2 CLOUD" else "DarkCyan"
        )
        self.tech_button_model3_cloud.configure(
            fg_color=("gray75", "gray25") if button == "Modelo 3 CLOUD" else "DarkCyan"
        )

        if button == "yolo":
            model = 1
        elif button == "Modelo 2 YOLO":
            model = 2
        elif button == "Modelo 3 YOLO":
            model = 3
        elif button == "hog":
            model = 4
        elif button == "Modelo 2 HOG":
            model = 5
        elif button == "Modelo 3 HOG":
            model = 6
        elif button == "cloud":
            model = 7
        elif button == "Modelo 2 CLOUD":
            model = 8
        elif button == "Modelo 3 CLOUD":
            model = 9
        else:
            pass

    """                                            MENU BUTTON EVENTS                                 
            COMPONENTS: home_button_event(), techniques_button_event(), models_button_event(), config_button_event(),
                        solution_button_event()
            PURPOSE OF THE FUNCTION: User possible interactions with the Left main menu
    """

    # Jumps to the init frame
    def home_button_event(self):
        self.select_frame_by_name("home")

    # Jumps to the selection technique frame
    def techniques_button_event(self):
        self.select_frame_by_name("frame_2")

    # Jumps to the selection model frame of the selected technique
    def models_button_event(self):
        if val == 1:
            self.select_frame_by_name("classique")
        elif val == 2:
            self.select_frame_by_name("neural")
        elif val == 3:
            self.select_frame_by_name("cloud")
        else:
            pass

    # Jumps to the configuration frame of the selected model
    def config_button_event(self):
        if model == 1:
            self.select_frame_by_name("yolo")
        elif model == 4:
            self.select_frame_by_name("hog")
        elif model == 7:
            self.select_frame_by_name("google")
        else:
            pass

    # Jumps to the final frame
    def solution_button_event(self):
        self.select_frame_by_name("frame_5")

    """                                            INIT FRAME BUTTON EVENTS                                
            COMPONENTS: start_button_event()
            PURPOSE OF THE FUNCTION: Pressing this button, the user starts the configuration process of the vision system
    """

    def start_button_event(self):
        self.select_frame_by_name("frame_2")

    """                                            TECHNIQUES FRAME BUTTON EVENTS                                 
            COMPONENTS: classique_button_event(), neural_button_event(), google_button_event(), next_button_event_1()
                        before_button_event_1()
            PURPOSE OF THE FUNCTION: User possible interactions with the Technique Selection Frame
    """

    # Selection of the Classique methods
    def classique_button_event(self):
        self.select_technique_by_button("classique")

    # Selection of the Neural Network methods
    def neural_button_event(self):
        self.select_technique_by_button("neural")

    # Selection of the Cloud methods
    def google_button_event(self):
        self.select_technique_by_button("cloud")

    # This button jumps to the selected technique model selection frame
    def next_button_event_1(self):
        if val == 1:
            self.select_frame_by_name("classique")
        elif val == 2:
            self.select_frame_by_name("neural")
        elif val == 3:
            self.select_frame_by_name("cloud")
        else:
            pass

    # This button returns to the home frame
    def before_button_event_1(self):
        self.select_frame_by_name("home")

    """                                            MODELS FRAME BUTTON EVENTS (YOLO)                                 
            COMPONENTS: before_button_event_yolo(), next_button_event_yolo(), yolo_button_event(), m2_yolo_button_event(), 
                        m3_yolo_button_event(), right_button_models_yolo(), left_button_models_yolo(), 
            PURPOSE OF THE FUNCTION: User possible interactions with the Neural Network Models Selection Frame 
    """

    # Return to the Techniques frame
    def before_button_event_yolo(self):
        self.select_frame_by_name("frame_2")

    # If the model is implemented, advances to the configuration frame
    def next_button_event_yolo(self):
        if model == 1:
            self.select_frame_by_name("yolo")
        elif model == 2:
            pass
        elif model == 3:
            pass

    # Selection of the YOLOv5 method
    def yolo_button_event(self):
        self.select_model_by_button("yolo")

    # Select a not implemented model (future implementation)
    def m2_yolo_button_event(self):
        self.select_model_by_button("Modelo 2 YOLO")

    # Select a not implemented model (future implementation)
    def m3_yolo_button_event(self):
        self.select_model_by_button("Modelo 3 YOLO")

    # Button in case of more than 3 models
    def right_button_models_yolo(self):
        pass

    # Button in case of more than 3 models
    def left_button_models_yolo(self):
        pass

    """                                            MODELS FRAME BUTTON EVENTS (HOG)                                
            COMPONENTS: before_button_event_hog(), next_button_event_hog(), hog_button_event(), m2_hog_button_event(), 
                        m3_hog_button_event(), right_button_models_hog(), left_button_models_hog(), 
            PURPOSE OF THE FUNCTION: User possible interactions with the Classique Methods Models Selection Frame 
    """

    # Return to the Techniques frame
    def before_button_event_hog(self):
        self.select_frame_by_name("frame_2")

    # If the model is implemented, advances to the configuration frame
    def next_button_event_hog(self):
        if model == 4:
            self.select_frame_by_name("hog")
        elif model == 5:
            pass
        elif model == 6:
            pass

    # Selection of the HOG method
    def hog_button_event(self):
        self.select_model_by_button("hog")

    # Select a not implemented model (future implementation)
    def m2_hog_button_event(self):
        self.select_model_by_button("Modelo 2 HOG")

    # Select a not implemented model (future implementation)
    def m3_hog_button_event(self):
        self.select_model_by_button("Modelo 3 HOG")

    # Button in case of more than 3 models
    def right_button_models_hog(self):
        pass

    # Button in case of more than 3 models
    def left_button_models_hog(self):
        pass

    """                                            MODELS FRAME BUTTON EVENTS (CLOUD)                                
            COMPONENTS: before_button_event_cloud(), next_button_event_cloud(), cloud_button_event(), 
                        m2_cloud_button_event(), m3_cloud_button_event(), right_button_models_cloud(),
                        left_button_models_cloud()

            PURPOSE OF THE FUNCTION: User possible interactions with the Cloud Models Selection Frame 
    """

    # Return to the Techniques frame
    def before_button_event_cloud(self):
        self.select_frame_by_name("frame_2")

    # If the model is implemented, advances to the configuration frame
    def next_button_event_cloud(self):
        if model == 7:
            self.select_frame_by_name("google")
        elif model == 8:
            pass
        elif model == 9:
            pass

    # Selection of the Google Vision API method
    def cloud_button_event(self):
        self.select_model_by_button("cloud")

    # Select a not implemented model (future implementation)
    def m2_cloud_button_event(self):
        self.select_model_by_button("Modelo 2 CLOUD")

    # Select a not implemented model (future implementation)
    def m3_cloud_button_event(self):
        self.select_model_by_button("Modelo 3 CLOUD")

    # Button in case of more than 3 models
    def right_button_models_cloud(self):
        pass

    # Button in case of more than 3 models
    def left_button_models_cloud(self):
        pass

    """                                            MODELS FRAME BUTTON EVENT (CLOUD, HOG, YOLOV5)                                
            COMPONENTS: before_button_event_conf(), next_button_event_conf()
            PURPOSE OF THE FUNCTION: This is common to all the configuration frames. In this functions, the program
                                     manages the information provided by the user in the different frames. It also
                                     allows the user to return to the models frame.
    """

    def before_button_event_conf(self):
        if val == 1:
            self.select_frame_by_name("classique")
        elif val == 2:
            self.select_frame_by_name("neural")
        elif val == 3:
            self.select_frame_by_name("cloud")
        else:
            pass

    # In this button event, the fields covered by the user with the paths and instructions for the vision systems
    # are managed, and saved in a .plmpy file
    # Here the final python script is structured too.
    def next_button_event_conf(self):

        # Read the plantillas

        # path and name files
        templates_path = "templates"

        # If the selected method is Yolov5
        if model == 1:

            # creation of the solution file
            solutions_path = "solutions"
            filename = "yolo_solution.py"
            file_path = os.path.join(solutions_path, filename)
            solution_file = open(file_path, "w")

            # path and name files
            init_template = "yolo_init.tplpy"
            training_template = "yolo_training.tplpy"
            inference_template = "yolo_inference.tplpy"

            # Opens the init file
            init_file = open(os.path.join(templates_path, init_template))
            init_str = init_file.read()

            # Read the instructions needed to configure the vision system writted by the user
            epochs_num_user = self.number_epochs_yolov5.get()
            result_dir = self.path_result_yolov5.get()  # deberia de haber un campo para resultados
            data_dir = self.path_train_yolov5.get()
            seed_number = self.number_seed_yolov5.get()
            inference_dir = self.path_inference_yolov5.get()

            # Manage the instructions needed to configure the vision system writted by the user
            init_str = init_str.replace("{/epoch_number/}", epochs_num_user)
            init_str = init_str.replace("{/resul_dir/}", result_dir)
            init_str = init_str.replace("{/data_dir/}", data_dir)
            init_str = init_str.replace("{/number_seed/}", seed_number)
            init_str = init_str.replace("{/infer_dir/}", inference_dir)

            # Write a heading for the script and the init file in the final file
            solution_file.write('"""Sistema de reconocimiento de objetos con YOLOv5"""\n\n')
            solution_file.write(init_str)
            solution_file.write("\n")

            # Open the training file
            training_file = open(os.path.join(templates_path, training_template))
            training_file_str = training_file.read()

            # Prints a message and the training file in the final file
            solution_file.write("#Entrenamiento\n")
            solution_file.write(training_file_str)
            solution_file.write("\n")

            # Open the inference file
            inference_file = open(os.path.join(templates_path, inference_template))
            inference_file_str = inference_file.read()

            # Prints a message and the inference file in the final file
            solution_file.write("#Inference\n")
            solution_file.write(inference_file_str)
            solution_file.write("\n")

        # If the selected method is HOG
        elif model == 4:

            # creation of the solution file
            solutions_path = "solutions"
            filename = "hog_solution.py"
            file_path = os.path.join(solutions_path, filename)
            solution_file = open(file_path, "w")

            # path and name files
            init_template = "hog_init.tplpy"
            training_template = "hog_training.tplpy"
            dataset_template = "hog_dataset.tplpy"
            results_visualization_template = "hog_results_visualization.tplpy"

            # Opens the init file
            init_file = open(os.path.join(templates_path, init_template))
            init_str = init_file.read()

            # Write a heading for the script and the init file in the final file
            solution_file.write('"""Sistema de reconocimiento de objetos con HOG"""\n\n')
            solution_file.write(init_str)
            solution_file.write("\n")

            # Opens the dataset file
            dataset_file = open(os.path.join(templates_path, dataset_template))
            dataset_file_str = dataset_file.read()

            # Read the instructions needed to configure the vision system writted by the user
            data_dir = self.path_train_hog.get()
            result_dir = self.path_result_hog.get()
            sel_classifier = self.selection_classifier_hog.get()
            test_size = self.test_size_hog.get()

            # Manage the instructions needed to configure the vision system writted by the user
            dataset_file_str = dataset_file_str.replace("{/data_dir/}", data_dir)
            dataset_file_str = dataset_file_str.replace("{/resul_dir/}", result_dir)
            dataset_file_str = dataset_file_str.replace("{/sel_clas/}", sel_classifier)
            dataset_file_str = dataset_file_str.replace("{/test_size/}", test_size)

            # Write a heading for the script and the init file in the final file
            solution_file.write("\n")
            solution_file.write("#Obtencion del dataset y de los paths \n")
            solution_file.write(dataset_file_str)
            solution_file.write("\n")

            # Open the training file
            training_file = open(os.path.join(templates_path, training_template))
            training_file_str = training_file.read()

            # Prints a message and the training file in the final file
            solution_file.write("\n")
            solution_file.write("#Entrenamiento \n")
            solution_file.write(training_file_str)
            solution_file.write("\n")

            # Open the results file
            results_file = open(os.path.join(templates_path, results_visualization_template))
            results_file_str = results_file.read()

            # Prints a message and the training file in the final file
            solution_file.write("\n")
            solution_file.write("#Result visualization \n")
            solution_file.write(results_file_str)
            solution_file.write("\n")

        # If the selected method is Google Vision API
        elif model == 7:

            # creation of the solution file
            solutions_path = "solutions"
            filename = "cloud_solution.py"
            file_path = os.path.join(solutions_path, filename)
            solution_file = open(file_path, "w")

            # path and name files
            init_template = "google_vision_API_init.tplpy"
            training_template = "google_vision_API_training.tplpy"
            dataset_template = "google_vision_API_dataset.tplpy"
            results_visualization_template = "google_vision_API_results_visualization.tplpy"
            main_template = "google_vision_API_main.tplpy"

            # Opens the init file
            init_file = open(os.path.join(templates_path, init_template))
            init_str = init_file.read()

            # Write a heading for the script and the init file in the final file
            solution_file.write('"""Sistema de reconocimiento de objetos con Google Vision API"""\n\n')
            solution_file.write(init_str)
            solution_file.write("\n")

            # Open the dataset file
            dataset_file = open(os.path.join(templates_path, dataset_template))
            dataset_file_str = dataset_file.read()

            # Read the instructions needed to configure the vision system writted by the user
            object_1 = self.object_1_cloud.get()
            object_2 = self.object_2_cloud.get()
            data_dir = self.path_train_cloud.get()
            result_dir = self.path_result_cloud.get()

            # Manage the instructions needed to configure the vision system writted by the user
            dataset_file_str = dataset_file_str.replace("{/data_dir/}", data_dir)
            dataset_file_str = dataset_file_str.replace("{/resul_dir/}", result_dir)
            dataset_file_str = dataset_file_str.replace("{/object_1/}", object_1)
            dataset_file_str = dataset_file_str.replace("{/object_2/}", object_2)

            # Prints a message and the training file in the final file
            solution_file.write("\n")
            solution_file.write("#Datos usuario\n")
            solution_file.write(dataset_file_str)
            solution_file.write("\n")

            # Open the training file
            training_file = open(os.path.join(templates_path, training_template))
            training_file_str = training_file.read()

            # Prints a message and the training file in the final file
            solution_file.write("\n")
            solution_file.write("#Entrenamiento\n")
            solution_file.write(training_file_str)
            solution_file.write("\n")

            # Open the result visualization file
            results_visualization_file = open(os.path.join(templates_path, results_visualization_template))
            results_visualization_file_str = results_visualization_file.read()

            # Prints a message and the training file in the final file
            solution_file.write("\n")
            solution_file.write("#Visualización de resultados\n")
            solution_file.write(results_visualization_file_str)
            solution_file.write("\n")

            # Open the main loop file
            main_file = open(os.path.join(templates_path, main_template))
            main_file_str = main_file.read()

            # Prints a message and the training file in the final file
            solution_file.write("\n")
            solution_file.write("#Bucle principal de la aplicación\n")
            solution_file.write(main_file_str)
            solution_file.write("\n")

        else:
            pass

        # Close the final file and jump to the solution frame
        solution_file.close()

        self.select_frame_by_name("frame_5")

    """                                            CHANGE THE APPEARANCE OF THE APP                                
            COMPONENTS: change_appearance_mode_event()
            PURPOSE OF THE FUNCTION: Changes the appearance from light to dark and viceversa.
    """

    def change_appearance_mode_event(self, new_appearance_mode):
        customtkinter.set_appearance_mode(new_appearance_mode)


"""                                            MAIN LOOP                                
            COMPONENTS: App(), mainloop().
            PURPOSE OF THE FUNCTION: Runs and process the frame events and user interactions
"""
if __name__ == "__main__":
    app = App()
    app.mainloop()
