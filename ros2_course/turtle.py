# This Python file uses the following encoding: utf-8
from PySide2 import QtWidgets
from turtle import Screen, Turtle
import random
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32

class TurtleSierpinski(Node):
    def __init__(self):
        super().__init__('turtle_sierpinski')
        self.size = 1

        self.declare_parameter('sierpinski_size', self.size)
        self.size = self.get_parameter('sierpinski_size').value

        self.screen = Screen()
        self.screen.setup(800, 600)
        self.screen.setworldcoordinates(0, 0, 800, 600)

        self.turtle = Turtle()
        self.turtle.hideturtle()
        self.turtle.penup()



        self.publisher = self.create_publisher(Int32, 'sierpinski_size', 10)

        self.timer = self.create_timer(1.0, self.publish_size)

        self.sierpinski(int(self.size / 2), int(self.size / 2), self.size)

    def draw_triangle(self, x, y, size):
        self.turtle.goto(x, y)
        self.screen.colormode(255)
        r = random.randint(0, 255)
        g = random.randint(0, 255)
        b = random.randint(0, 255)
        self.turtle.pencolor(int(r), int(g), int(b))
        self.turtle.fillcolor(int(r), int(g), int(b))

        self.turtle.pendown()
        self.turtle.begin_fill()
        for _ in range(3):
            self.turtle.forward(size)
            self.turtle.left(120)
        self.turtle.end_fill()
        self.turtle.penup()



    def sierpinski(self, x, y, size):
        self.draw_triangle(x, y, size)

        if size > 10:
            self.sierpinski(x, y + int(size / 2), int(size / 2))
            self.sierpinski(x + int(size / 2), y - int(size / 2), int(size / 2))
            self.sierpinski(x - int(size / 2), y - int(size / 2), int(size / 2))

        if size == self.size:
            self.screen.bgcolor("gray")

    def publish_size(self):
        msg = Int32()
        msg.data = self.size
        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    turtle_sierpinski = TurtleSierpinski()
    rclpy.spin(turtle_sierpinski)
    turtle_sierpinski.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
