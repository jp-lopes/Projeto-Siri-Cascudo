import numpy as np

class color:
    name = ""
    lower = np.array([-1,-1,-1])
    upper = np.array([-1,-1,-1])

    def __init__(self, name, lower, upper):
        self.name = name
        self.lower = lower
        self.upper = upper

# Preto, cinza, branco
black   = color("Black",   np.array([0, 0, 0]),       np.array([180, 255, 50]))
gray    = color("Gray",    np.array([0, 0, 50]),      np.array([180, 50, 200]))
white   = color("White",   np.array([0, 0, 200]),     np.array([180, 30, 255]))
# Vermelhos
red1    = color("Red",    np.array([0, 50, 50]),     np.array([10, 255, 255]))
red2    = color("Red",    np.array([170, 50, 50]),   np.array([180, 255, 255]))
# Tons de laranja/amarelo/bege
orange  = color("Orange",  np.array([10, 50, 50]),    np.array([20, 255, 255]))
beige   = color("Beige",   np.array([20, 30, 150]),  np.array([30, 100, 255]))
yellow  = color("Yellow",  np.array([25, 50, 50]),   np.array([35, 255, 255]))
# Verdes
green        = color("Green",       np.array([35, 50, 50]),   np.array([65, 255, 255]))
light_green  = color("LightGreen",  np.array([50, 30, 150]),  np.array([65, 150, 255]))
# Ciano / tons de azul
cyan       = color("Cyan",      np.array([65, 50, 50]),   np.array([85, 255, 255]))
light_blue = color("LightBlue", np.array([85, 50, 150]),  np.array([100, 150, 255]))
blue       = color("Blue",      np.array([100, 50, 50]),  np.array([130, 255, 255]))
# Roxos / magentas / rosa
purple  = color("Purple",  np.array([130, 50, 50]),  np.array([150, 255, 255]))
magenta = color("Magenta", np.array([150, 50, 50]),  np.array([160, 255, 255]))
pink    = color("Pink",    np.array([160, 50, 150]), np.array([170, 150, 255]))
# Marrom
brown = color("Brown", np.array([10, 50, 50]), np.array([20, 255, 150]))

# Lista de cores completa 
colors_list = [black, gray, white, red1, red2, orange, beige, yellow, green, light_green, 
               cyan, light_blue, blue, purple, magenta, pink, brown]