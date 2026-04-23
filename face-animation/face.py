import turtle
import time
import random

# -------------------------
# Screen setup
# -------------------------
win = turtle.Screen()
win.title("Simple Face Animation")
win.bgcolor("white")
win.setup(width=1024, height=600)
win._root.attributes("-fullscreen", True)

# -------------------------
# Eyes
# -------------------------
EYE_HEIGHT = 5
EYE_WIDTH = 5  

left_eye = turtle.Turtle()
left_eye.shape("circle")
left_eye.color("black")
left_eye.penup()
left_eye.goto(-200, 170)
left_eye.shapesize(EYE_HEIGHT, EYE_WIDTH)

right_eye = turtle.Turtle()
right_eye.shape("circle")
right_eye.color("black")
right_eye.penup()
right_eye.goto(200, 170)
right_eye.shapesize(EYE_HEIGHT, EYE_WIDTH)

# -------------------------
# Mouth
# -------------------------
mouth = turtle.Turtle()
mouth.hideturtle()
mouth.color("black")
mouth.pensize(40)
mouth.penup()
mouth.goto(-250, -50)
mouth.pendown()
mouth.setheading(-60)
mouth.circle(300, 120)

# -------------------------
# Blink function
# -------------------------
def blink():
    # close
    left_eye.shapesize(0.2, EYE_WIDTH)
    right_eye.shapesize(0.2, EYE_WIDTH)
    time.sleep(0.12)

    # open
    left_eye.shapesize(EYE_HEIGHT, EYE_WIDTH)
    right_eye.shapesize(EYE_HEIGHT, EYE_WIDTH)

# -------------------------
# Main loop (random blinking)
# -------------------------
while True:
    time.sleep(random.uniform(1.5, 5))  # random pause between blinks
    
    # sometimes do a double blink
    blink()
    if random.random() < 0.3:
        time.sleep(0.15)
        blink()